/*
 * espsdrtx_mod: packet modulators. The API and the packet format are in
 * espsdrtx_mod.h.
 *
 * Each modulator works the same way:
 *   1. packet_bytes builds SIGNAL and the frame (the bytes every PHY
 *      carries);
 *   2. the bytes become symbols (constellation points, frequency levels or
 *      chirp values);
 *   3. the samples are computed one at a time and handed to the caller's
 *      sink through PUT, in order k = 0, 1, ..., so the whole waveform never
 *      has to be held in memory.
 * PUT leaves the function through `fail` (freeing what it allocated and
 * returning 0) as soon as the sink refuses a sample.
 *
 * Bits are sent LSB first throughout. The C3 has no FPU, so phases are
 * wrapped before every cosf/sinf (cosf of a large angle takes about 0.2 ms
 * there), and filters are tables or running sums.
 */
#include "espsdrtx_mod.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "espsdrtx_tables.h"

#define TWO_PI		6.283185307179586

/*
 * The packet. SIGNAL is SIG_BYTES bytes (SIG_BITS bits); the frame adds
 * FRAME_EXTRA bytes to the payload: a sequence byte before it and a CRC-32
 * after it. The frame length is sent in 16 bits.
 */
#define SIG_BYTES	6
#define SIG_BITS	(SIG_BYTES * 8)
#define FRAME_EXTRA	5
#define FRAME_MAX	0xffff

/* CRC-32 as zlib computes it: the reflected form of polynomial 0x04c11db7. */
#define CRC32_POLY	0xedb88320u

/* Root-raised-cosine roll-off of the single-carrier pulse. */
#define RRC_BETA	0.35
/* How close to one of the RRC formula's 0 / 0 points counts as on it. */
#define RRC_EPS		1e-9

/* ------------------------------------------------------- shared helpers */

uint32_t espsdrtx_crc32(const uint8_t *b, size_t n)
{
	/* Bitwise, one bit per step: slow, but needs no table. */
	uint32_t c = 0xffffffffu;

	while (n--) {
		c ^= *b++;
		/* 0u - (c & 1) is all ones if the low bit is set: XOR then. */
		for (int k = 0; k < 8; k++)
			c = (c >> 1) ^ (CRC32_POLY & (0u - (c & 1u)));
	}
	return ~c;
}

/* Bit k of the byte string b, LSB first: byte k / 8, bit k % 8. */
static int bit(const uint8_t *b, size_t k)
{
	return (b[k >> 3] >> (k & 7)) & 1;
}

/*
 * Inverse Gray code. A symbol's m-bit value v sits at position gray_inv(v)
 * of the constellation (or of the frequency or chirp levels), so
 * neighbouring positions differ in one bit.
 */
static unsigned int gray_inv(unsigned int g)
{
	unsigned int b = g;

	for (g >>= 1; g; g >>= 1)
		b ^= g;
	return b;
}

/* m bits starting at bit k of b (nbits long, zeros beyond it), LSB first. */
static unsigned int bits_at(const uint8_t *b, size_t nbits, size_t k, int m)
{
	unsigned int v = 0;

	for (int j = 0; j < m; j++)
		if (k + j < nbits)
			v |= (unsigned int)bit(b, k + j) << j;
	return v;
}

/*
 * SIGNAL and the frame in one buffer, which the caller frees (NULL if too
 * long or out of memory). *frame_len gets the frame's length, from byte 6.
 *   bytes 0..5  SIGNAL: m (bits per symbol), the frame length (2 bytes),
 *               CRC-32 of those 3 (its low 3 bytes)
 *   bytes 6..   frame: seq, payload, CRC-32 of seq and payload (4 bytes)
 * All multi-byte fields are little-endian.
 */
static uint8_t *packet_bytes(const uint8_t *payload, size_t len, uint8_t seq,
			     uint8_t m, size_t *frame_len)
{
	if (len > FRAME_MAX - FRAME_EXTRA)
		return NULL;	/* the frame length must fit 16 bits */
	size_t fl = len + FRAME_EXTRA;
	uint8_t *b = malloc(SIG_BYTES + fl);

	if (!b)
		return NULL;
	uint8_t *f = b + SIG_BYTES;

	f[0] = seq;
	memcpy(f + 1, payload, len);
	uint32_t c = espsdrtx_crc32(f, (size_t)len + 1);

	for (int k = 0; k < 4; k++)
		f[len + 1 + k] = (uint8_t)(c >> (8 * k));
	b[0] = m;
	b[1] = (uint8_t)fl;
	b[2] = (uint8_t)(fl >> 8);
	c = espsdrtx_crc32(b, 3);
	for (int k = 0; k < 3; k++)
		b[3 + k] = (uint8_t)(c >> (8 * k));
	*frame_len = fl;
	return b;
}

/*
 * Constellation point of the m-bit value v (its first bit in bit 0), unit
 * mean power:
 *   m 1: BPSK, bit 1 = -1;
 *   m 3: 8-PSK, at angle gray_inv(v) * 45 degrees;
 *   m 2, 4, 6: square QAM, the low m/2 bits Gray-coding I and the high ones
 *   Q.
 */
static void qam_point(int m, unsigned int v, float *re, float *im)
{
	if (m == 1) {
		*re = v & 1 ? -1.0f : 1.0f;
		*im = 0;
	} else if (m == 3) {
		double a = TWO_PI * gray_inv(v) / 8;

		*re = (float)cos(a);
		*im = (float)sin(a);
	} else {
		/*
		 * L levels per axis, at -(L-1), ..., -1, 1, ..., L-1. Their
		 * mean square is (L^2 - 1) / 3 per axis, so dividing by the
		 * root of twice that gives unit power.
		 */
		int h = m / 2, L = 1 << h;
		unsigned int i = gray_inv(v & ((1u << h) - 1));
		unsigned int q = gray_inv(v >> h);
		float norm = sqrtf(2.0f * (L * L - 1) / 3);

		*re = (2.0f * i - (L - 1)) / norm;
		*im = (2.0f * q - (L - 1)) / norm;
	}
}

static int iround(double x)
{
	return (int)lround(x);
}

/* Hand sample k to the sink; on refusal, clean up and return 0. */
#define PUT(k, i, q)					\
	do {						\
		if (put(ctx, (k), (i), (q)))		\
			goto fail;			\
	} while (0)

/* -------------------------------------------------------- single carrier
 *
 * Symbols: SC_PRE preamble (QPSK, fixed), SC_SIG SIGNAL (QPSK: 48 bits), the
 * frame at m bits per symbol, SC_GAP silent. Each symbol is an impulse every
 * sps samples, shaped by a root-raised-cosine pulse spanning +-SC_SPAN
 * symbols.
 */

#define SC_PRE		64	/* preamble symbols */
#define SC_SIG		24	/* SIGNAL symbols: 6 bytes in QPSK */
#define SC_GAP		16	/* silent symbols at the end */
#define SC_SPAN		8	/* pulse half-length, symbols */
#define SC_HEAD		(SC_PRE + SC_SIG)	/* the symbols in QPSK */
#define SC_POINTS	64	/* the largest constellation, 64-QAM */
#define SC_SILENT	SC_POINTS	/* index of the zero "point" */

/*
 * Root-raised-cosine pulse, roll-off beta, x in symbols; the special cases
 * are the formula's 0 / 0 points, at x = 0 and |x| = 1 / (4 beta).
 */
static double rrc(double x, double beta)
{
	if (fabs(x) < RRC_EPS)
		return 1 - beta + 4 * beta / M_PI;
	if (fabs(fabs(x) - 1 / (4 * beta)) < RRC_EPS)
		return beta / sqrt(2) *
		       ((1 + 2 / M_PI) * sin(M_PI / (4 * beta)) +
			(1 - 2 / M_PI) * cos(M_PI / (4 * beta)));
	return (sin(M_PI * x * (1 - beta)) +
		4 * beta * x * cos(M_PI * x * (1 + beta))) /
	       (M_PI * x * (1 - (4 * beta * x) * (4 * beta * x)));
}

/*
 * Sample k: the sum of the pulses of the symbols within SC_SPAN of it. The
 * waveform plays in a loop, so symbol indices wrap around (jj mod nsym): the
 * pulses at the end overlap the start, and the loop joins without a jump.
 * points[0] is QPSK, for the preamble and SIGNAL; points[1] the frame's.
 */
static void sc_sample(uint32_t k, int sps, size_t nsym, const uint8_t *sym,
		      float points[2][SC_POINTS + 1][2], const float *tap,
		      float *re, float *im)
{
	long c = (long)(k / sps);	/* the symbol sample k falls in */

	*re = 0;
	*im = 0;
	for (long jj = c - SC_SPAN; jj <= c + SC_SPAN + 1; jj++) {
		long d = (long)k - jj * sps;	/* from symbol jj's centre */

		if (d < -SC_SPAN * sps || d > SC_SPAN * sps)
			continue;
		size_t j = (size_t)((jj % (long)nsym + (long)nsym) %
				    (long)nsym);
		const float *p = points[j < SC_HEAD ? 0 : 1][sym[j]];
		float h = tap[d + SC_SPAN * sps];

		*re += p[0] * h;
		*im += p[1] * h;
	}
}

uint32_t espsdrtx_sc_packet(const uint8_t *payload, size_t len, uint8_t seq,
			    int m, int sps, float rms, uint32_t max_n,
			    espsdrtx_put_t put, void *ctx)
{
	if (!(m == 1 || m == 2 || m == 3 || m == 4 || m == 6) || sps < 2 ||
	    (uint32_t)sps > max_n)
		return 0;
	size_t fl;
	uint8_t *pb = packet_bytes(payload, len, seq, (uint8_t)m, &fl);

	if (!pb)
		return 0;
	size_t nframe = (fl * 8 + m - 1) / m;	/* frame symbols, rounded up */
	size_t nsym = SC_HEAD + nframe + SC_GAP;
	/* n is 0 if the packet does not fit max_n. */
	uint32_t n = nsym <= max_n / (uint32_t)sps ? (uint32_t)(nsym * sps) : 0;
	uint32_t ntap = 2 * SC_SPAN * sps + 1;
	uint32_t ret = 0;
	uint8_t *sym = n ? malloc(nsym) : NULL;
	float *tap = sym ? malloc(ntap * sizeof(float)) : NULL;

	if (!tap)
		goto fail;
	/*
	 * Two constellations: points[0] QPSK; points[1] the frame's, plus the
	 * silent symbol at SC_SILENT. sym[j] is symbol j's index into its
	 * constellation.
	 */
	float points[2][SC_POINTS + 1][2];

	for (unsigned int v = 0; v < 4; v++)
		qam_point(2, v, &points[0][v][0], &points[0][v][1]);
	for (unsigned int v = 0; v < (1u << m); v++)
		qam_point(m, v, &points[1][v][0], &points[1][v][1]);
	points[1][SC_SILENT][0] = points[1][SC_SILENT][1] = 0;
	for (size_t j = 0; j < nsym; j++) {
		if (j < SC_PRE)
			sym[j] = (uint8_t)bits_at(espsdrtx_preamble_bits,
						  2 * SC_PRE, 2 * j, 2);
		else if (j < SC_HEAD)
			sym[j] = (uint8_t)bits_at(pb, SIG_BITS,
						  2 * (j - SC_PRE), 2);
		else if (j < SC_HEAD + nframe)
			sym[j] = (uint8_t)bits_at(pb + SIG_BYTES, fl * 8,
						  m * (j - SC_HEAD), m);
		else
			sym[j] = SC_SILENT;
	}
	/*
	 * Pulse taps, tap[t] at t - SC_SPAN sps samples from the pulse centre,
	 * scaled to a sum of squares of sps rms^2: one symbol of unit power
	 * per sps samples then comes out at the requested rms.
	 */
	double e = 0;

	for (uint32_t t = 0; t < ntap; t++) {
		double h = rrc(((double)t - SC_SPAN * sps) / sps, RRC_BETA);

		tap[t] = (float)h;
		e += h * h;
	}
	float scale = (float)(rms * sqrt((double)sps / e));

	for (uint32_t t = 0; t < ntap; t++)
		tap[t] *= scale;
	for (uint32_t k = 0; k < n; k++) {
		float re, im;

		sc_sample(k, sps, nsym, sym, points, tap, &re, &im);
		PUT(k, iround(re), iround(im));
	}
	ret = n;
fail:
	free(tap);
	free(sym);
	free(pb);
	return ret;
}

/* ------------------------------------------------------------------ OFDM
 *
 * 802.11a numerology: 64 subcarriers 312.5 kHz apart (a 64-point FFT at
 * 20 MS/s), a 16-sample cyclic prefix, 52 used subcarriers (-26..26 without
 * 0), pilots on -21, -7, 7, 21, and 48 data subcarriers. Built 4x
 * oversampled for 80 MS/s: a 256-point inverse FFT with the subcarriers
 * outside -26..26 left at zero.
 *
 * Layout, in 20 MS/s samples (times OF_OS at 80 MS/s):
 *   OF_GAP silent symbols   (OF_N + OF_CP each)
 *   LTF: a 2 OF_CP prefix, then the training symbol twice
 *        (2 OF_CP + 2 OF_N)
 *   SIGNAL, one BPSK symbol   (OF_CP + OF_N)
 *   data symbols, 48 subcarriers of m bits each   (OF_CP + OF_N each)
 * SIGNAL and the frame are scrambled (XOR with espsdrtx_white_bits), each from
 * the sequence's start.
 */

#define OF_N		64		/* FFT size at 20 MS/s */
#define OF_CP		16		/* cyclic prefix */
#define OF_OS		4		/* oversampling: 80 MS/s */
#define OF_M		(OF_N * OF_OS)	/* the FFT size actually computed */
#define OF_GAP		2		/* silent symbols at the start */
#define OF_EDGE		26		/* used subcarriers: -26..-1, 1..26 */
#define OF_USED		(2 * OF_EDGE)	/* 52 */
#define OF_DATA		48		/* data subcarriers per symbol */
#define OF_NPILOT	4
#define OF_PILOT_SEQ	256		/* the pilot signs repeat after this */

static const int8_t of_pilots[OF_NPILOT] = {-21, -7, 7, 21};

static bool of_is_pilot(int c)
{
	return c == -21 || c == -7 || c == 7 || c == 21;
}

static void swap(float *a, float *b)
{
	float t = *a;

	*a = *b;
	*b = t;
}

/*
 * In-place inverse DFT of OF_M points, unnormalised:
 * x[n] = sum X[k] e^{+2 pi i k n / OF_M}.
 * Radix-2: bit-reversal permutation, then butterflies.
 */
static void ifft(float *re, float *im)
{
	/*
	 * Permutation: j runs through the bit-reversed counts of i; the
	 * inner loop adds 1 to j from the top bit down (carrying through the
	 * set bits).
	 */
	for (unsigned int i = 1, j = 0; i < OF_M; i++) {
		unsigned int b = OF_M >> 1;

		for (; j & b; b >>= 1)
			j ^= b;
		j |= b;
		if (i < j) {
			swap(&re[i], &re[j]);
			swap(&im[i], &im[j]);
		}
	}
	/* Twiddles e^{2 pi i k / OF_M}, built on first use. */
	static float tw[OF_M / 2][2];

	if (tw[0][0] == 0)	/* tw[0] is 1 once built */
		for (unsigned int k = 0; k < OF_M / 2; k++) {
			tw[k][0] = (float)cos(TWO_PI * k / OF_M);
			tw[k][1] = (float)sin(TWO_PI * k / OF_M);
		}
	/*
	 * Butterflies over blocks of len points: point a and point b, half a
	 * block apart, become a + w b and a - w b, w the block's k-th
	 * twiddle.
	 */
	for (unsigned int len = 2; len <= OF_M; len <<= 1) {
		for (unsigned int k = 0; k < len / 2; k++) {
			float wr = tw[k * (OF_M / len)][0];
			float wi = tw[k * (OF_M / len)][1];

			for (unsigned int s = 0; s < OF_M; s += len) {
				unsigned int a = s + k, b = a + len / 2;
				float xr = re[b] * wr - im[b] * wi;
				float xi = re[b] * wi + im[b] * wr;

				re[b] = re[a] - xr;
				im[b] = im[a] - xi;
				re[a] += xr;
				im[a] += xi;
			}
		}
	}
}

/* Subcarrier k (negative k wraps to the top of the FFT input). */
static void of_set(float *re, float *im, int k, float vr, float vi)
{
	unsigned int i = (unsigned int)(k + OF_M) % OF_M;

	re[i] = vr;
	im[i] = vi;
}

/*
 * The value of data subcarrier d (0..47) in OFDM symbol row. Row 0 is
 * SIGNAL: its 48 bits in BPSK. Later rows carry the frame, m bits per
 * subcarrier; subcarriers past its end are zero, and so are bits past its
 * end in the last one.
 */
static void of_cell(const uint8_t *pb, size_t nbits, size_t ncells,
		    size_t row, int d, int m, float *vr, float *vi)
{
	*vr = 0;
	*vi = 0;
	if (row == 0) {
		*vr = bit(pb, d) ^ bit(espsdrtx_white_bits, d) ? -1.0f : 1.0f;
		return;
	}
	size_t s = (row - 1) * OF_DATA + d;	/* the cell's number */

	if (s >= ncells)
		return;
	unsigned int v = 0;

	for (int j = 0; j < m; j++) {
		size_t q = s * m + j;

		if (q < nbits)
			v |= (unsigned int)(bit(pb + SIG_BYTES, q) ^
					    bit(espsdrtx_white_bits, q)) << j;
	}
	qam_point(m, v, vr, vi);
}

uint32_t espsdrtx_ofdm_packet(const uint8_t *payload, size_t len,
			      uint8_t seq, int m, float rms, uint32_t max_n,
			      espsdrtx_put_t put, void *ctx)
{
	if (!(m == 1 || m == 2 || m == 4 || m == 6))
		return 0;
	size_t fl;
	uint8_t *pb = packet_bytes(payload, len, seq, (uint8_t)m, &fl);

	if (!pb)
		return 0;
	size_t nbits = fl * 8;			/* frame bits */
	size_t ncells = (nbits + m - 1) / m;	/* data subcarriers they fill */
	/* OFDM symbols after the LTF: SIGNAL, then the data, rounded up. */
	size_t rows = 1 + (ncells + OF_DATA - 1) / OF_DATA;
	uint32_t n = (uint32_t)(((OF_GAP + rows) * (OF_N + OF_CP) +
				 2 * OF_CP + 2 * OF_N) * OF_OS);
	uint32_t ret = 0;
	/* The scrambling sequence must cover the frame. */
	float *buf = nbits <= ESPSDRTX_WHITE_BITS && n <= max_n ?
		     malloc(4 * OF_M * sizeof(float)) : NULL;

	if (!buf)
		goto fail;
	/* The LTF symbol, kept for its two copies, then the current symbol. */
	float *lr = buf, *li = buf + OF_M;
	float *re = buf + 2 * OF_M, *im = buf + 3 * OF_M;
	/*
	 * Unit-power subcarriers make sum |X|^2 = 52 per symbol; the inverse
	 * FFT is unnormalised, so this sets the rms per sample.
	 */
	float scale = (float)(rms / sqrt((double)OF_USED));
	uint32_t k = 0;

	for (; k < OF_GAP * (OF_N + OF_CP) * OF_OS; k++)
		PUT(k, 0, 0);
	/*
	 * LTF: BPSK on all 52 subcarriers, sent as its last 2 OF_CP OF_OS
	 * samples, then twice in full.
	 */
	memset(lr, 0, 2 * OF_M * sizeof(float));
	for (int i = 0, c = -OF_EDGE; c <= OF_EDGE; c++)
		if (c)
			of_set(lr, li, c,
			       bit(espsdrtx_ltf_bits, i++) ? -1.0f : 1.0f, 0);
	ifft(lr, li);
	for (uint32_t t = OF_M - 2 * OF_CP * OF_OS; t < OF_M; t++, k++)
		PUT(k, iround(lr[t] * scale), iround(li[t] * scale));
	for (int r = 0; r < 2; r++)
		for (uint32_t t = 0; t < OF_M; t++, k++)
			PUT(k, iround(lr[t] * scale), iround(li[t] * scale));
	/*
	 * SIGNAL (row 0) and the data symbols: pilots with a per-symbol sign,
	 * data subcarriers from -26 up, skipping 0 and the pilots; then the
	 * prefix (the symbol's last OF_CP OF_OS samples) and the symbol.
	 */
	for (size_t row = 0; row < rows; row++) {
		memset(re, 0, 2 * OF_M * sizeof(float));
		float ps = bit(espsdrtx_pilot_bits, row % OF_PILOT_SEQ) ?
			   -1.0f : 1.0f;

		for (int p = 0; p < OF_NPILOT; p++)
			of_set(re, im, of_pilots[p], ps, 0);
		/* d: the data subcarrier's number, 0..47. */
		for (int d = 0, c = -OF_EDGE; c <= OF_EDGE; c++) {
			float vr, vi;

			if (!c || of_is_pilot(c))
				continue;
			of_cell(pb, nbits, ncells, row, d, m, &vr, &vi);
			of_set(re, im, c, vr, vi);
			d++;
		}
		ifft(re, im);
		for (uint32_t t = OF_M - OF_CP * OF_OS; t < OF_M; t++, k++)
			PUT(k, iround(re[t] * scale), iround(im[t] * scale));
		for (uint32_t t = 0; t < OF_M; t++, k++)
			PUT(k, iround(re[t] * scale), iround(im[t] * scale));
	}
	ret = n;
fail:
	free(buf);
	free(pb);
	return ret;
}

/* ------------------------------------------------------------------- FSK
 *
 * Each symbol is a frequency level (+-1, or +-1, +-3 for 4 levels), held for
 * sps samples. The levels may be Gaussian filtered (GFSK), and the phase
 * integrates them, so it never jumps. Layout: a FSK_GAP_US gap, then the
 * symbols listed at fsk_level, with the first and last symbol's amplitude
 * ramped.
 */

#define FSK_PRE		16	/* alternating preamble symbols */
#define FSK_SYNC	32	/* sync word symbols (bits) */
#define FSK_TAIL	4	/* alternating symbols at the end */
#define FSK_GAP_US	20	/* silence before the packet, microseconds */
#define FSK_BT_MIN	0.1f	/* see espsdrtx_fsk_packet */
#define GAUSS_SIGMAS	3	/* the filter is cut 3 sigma from its centre */

static const uint8_t fsk_sync[FSK_SYNC / 8] = {0x93, 0x0b, 0x51, 0xde};

struct fsk_src {
	const uint8_t *pb;	/* SIGNAL and frame, from packet_bytes */
	size_t fl;		/* frame length, bytes */
	int nlev;		/* 2 or 4 levels */
};

/*
 * Frequency level of symbol s:
 *   FSK_PRE preamble (alternating -1, +1), FSK_SYNC sync word, SIG_BITS
 *   SIGNAL (2 levels, bit 1 = +1),
 *   the frame (2 levels: one bit per symbol; 4 levels: two bits, first bit
 *   first, Gray: 00 -3, 10 -1, 11 +1, 01 +3),
 *   then alternating tail symbols.
 */
static int fsk_level(const struct fsk_src *q, size_t s)
{
	size_t nd = q->fl * 8 / (q->nlev == 4 ? 2 : 1);	/* frame symbols */

	if (s < FSK_PRE)
		return s & 1 ? 1 : -1;
	s -= FSK_PRE;
	if (s < FSK_SYNC)
		return 2 * bit(fsk_sync, s) - 1;
	s -= FSK_SYNC;
	if (s < SIG_BITS)
		return 2 * bit(q->pb, s) - 1;
	s -= SIG_BITS;
	if (s >= nd)
		return (s - nd) & 1 ? 1 : -1;
	if (q->nlev == 2)
		return 2 * bit(q->pb + SIG_BYTES, s) - 1;
	/*
	 * Two bits, the first in bit 0 of g. The position of g, 0..3, is
	 * g ^ (g >> 1) (for 2 bits, Gray and inverse Gray coincide), and
	 * positions 0..3 are the levels -3, -1, +1, +3.
	 */
	unsigned int g = (unsigned int)bit(q->pb + SIG_BYTES, 2 * s) |
			 (unsigned int)bit(q->pb + SIG_BYTES, 2 * s + 1) << 1;

	return 2 * (int)(g ^ (g >> 1)) - 3;
}

/*
 * The Gaussian filter's running sum, 2 half + 2 entries: G[u + 1] is the sum
 * of taps 0..u, normalised so the whole filter sums to 1 (G[0] = 0). A whole
 * symbol's share of a filtered sample is then a difference of two entries
 * instead of a sum over sps taps. NULL if out of memory.
 */
static double *gauss_cumsum(double sigma, long half)
{
	double *G = malloc((2 * half + 2) * sizeof(double));

	if (!G)
		return NULL;
	G[0] = 0;
	for (long u = 0; u <= 2 * half; u++)
		G[u + 1] = G[u] + exp(-(double)(u - half) * (u - half) /
				      (2 * sigma * sigma));
	double s = G[2 * half + 1];

	for (long u = 0; u <= 2 * half + 1; u++)
		G[u] /= s;
	return G;
}

/*
 * The filtered level at sample i of nmod: the "same" convolution of the
 * per-sample levels with the filter, taps over samples lo..hi (clipped to
 * the packet). Split into symbols, each symbol's samples a..b weighted by
 * the filter's sum over them.
 */
static double fsk_filtered(const struct fsk_src *q, const double *G,
			   long half, long sps, long nmod, long i)
{
	long lo = i - half < 0 ? 0 : i - half;
	long hi = i + half >= nmod ? nmod - 1 : i + half;
	double f = 0;

	for (long s = lo / sps; s <= hi / sps; s++) {
		long a = s * sps < lo ? lo : s * sps;
		long b = s * sps + sps - 1 > hi ? hi : s * sps + sps - 1;

		f += fsk_level(q, (size_t)s) *
		     (G[i + half - a + 1] - G[i + half - b]);
	}
	return f;
}

uint32_t espsdrtx_fsk_packet(const uint8_t *payload, size_t len, uint8_t seq,
			     float fs, float rate, float h, float bt, int nlev,
			     float amp, uint32_t max_n, espsdrtx_put_t put,
			     void *ctx)
{
	long sps = lround((double)fs / rate);

	/*
	 * bt 0 (rectangular) or >= FSK_BT_MIN: a narrower filter spans many
	 * symbols (slow here, and Python's "same" convolution changes length
	 * once the filter outgrows the packet).
	 */
	if ((nlev != 2 && nlev != 4) || sps < 2 || (uint32_t)sps > max_n ||
	    (double)sps * rate != fs || !(bt == 0 || bt >= FSK_BT_MIN))
		return 0;
	size_t fl;
	uint8_t *pb = packet_bytes(payload, len, seq, (uint8_t)(nlev / 2),
				   &fl);

	if (!pb)
		return 0;
	struct fsk_src q = {pb, fl, nlev};
	size_t nsym = FSK_PRE + FSK_SYNC + SIG_BITS +
		      fl * 8 / (nlev == 4 ? 2 : 1) + FSK_TAIL;
	/* Length in doubles first: a huge packet cannot overflow the check. */
	double gap_d = FSK_GAP_US * 1e-6 * (double)fs;
	double n_d = (uint32_t)gap_d + (double)nsym * sps;
	/* Modulated samples, after the gap. */
	size_t nmod = n_d <= max_n ? nsym * (size_t)sps : 0;
	uint32_t gap = (uint32_t)gap_d;
	uint32_t n = n_d <= max_n ? (uint32_t)n_d : max_n + 1;
	uint32_t ret = 0;
	/*
	 * Gaussian filter of bandwidth bt times the symbol rate. Its standard
	 * deviation, in samples: the -3 dB bandwidth B of a Gaussian with
	 * deviation sigma seconds is sqrt(ln 2) / (2 pi sigma).
	 */
	long half = 0;	/* filter half-length, samples */
	double *G = NULL;

	if (n <= max_n && bt > 0) {
		double sigma = sqrt(log(2.0)) / (TWO_PI * bt) * sps;

		half = (long)(GAUSS_SIGMAS * sigma) + 1;
		G = gauss_cumsum(sigma, half);
	}
	if (n > max_n || (bt > 0 && !G))
		goto fail;
	uint32_t k = 0;

	for (; k < gap; k++)
		PUT(k, 0, 0);
	/*
	 * Level 1 is a deviation of h rate / 2 Hz: this much phase per sample.
	 * (h is the modulation index: the peak-to-peak shift between levels
	 * -1 and +1 over the symbol rate.)
	 */
	double unit = TWO_PI * h * rate / 2 / fs, ph = 0;

	for (long i = 0; i < (long)nmod; i++, k++) {
		double f;	/* the (filtered) level at sample i */

		if (!G)
			f = fsk_level(&q, (size_t)(i / sps));
		else
			f = fsk_filtered(&q, G, half, sps, (long)nmod, i);
		ph += unit * f;
		if (ph > M_PI)
			ph -= TWO_PI;
		else if (ph < -M_PI)
			ph += TWO_PI;
		/* Raised-cosine amplitude ramps over the first, last symbol. */
		double r = amp;

		if (i < sps)
			r *= 0.5 - 0.5 * cos(M_PI * i / sps);
		else if (i >= (long)nmod - sps)
			r *= 0.5 - 0.5 * cos(M_PI * ((long)nmod - 1 - i) / sps);
		PUT(k, iround(r * cosf((float)ph)),
		    iround(r * sinf((float)ph)));
	}
	ret = n;
fail:
	free(G);
	free(pb);
	return ret;
}

/* ------------------------------------------------------------------- CSS
 *
 * Chirp spread spectrum: a chirp sweeps the frequency linearly from -bw/2 to
 * bw/2 over N = 2^sf chips. A chirp carrying value s starts s chips into the
 * sweep and wraps around, so it sends sf bits. Layout: a CSS_GAP_US gap,
 * CSS_UP upchirps and CSS_DOWN downchirps of value 0 (the preamble), then
 * SIGNAL and the frame as one bit stream, sf bits per chirp: the bits are
 * the Gray code of the chirp's value.
 */

#define CSS_UP		8	/* preamble upchirps */
#define CSS_DOWN	2	/* then downchirps */
#define CSS_PRE		(CSS_UP + CSS_DOWN)
#define CSS_GAP_US	10	/* silence before the packet, microseconds */
#define CSS_SF_MIN	2
#define CSS_SF_MAX	12

uint32_t espsdrtx_css_packet(const uint8_t *payload, size_t len, uint8_t seq,
			     float fs, float bw, int sf, float amp,
			     uint32_t max_n, espsdrtx_put_t put, void *ctx)
{
	long os = lround((double)fs / bw);	/* samples per chip */

	if (sf < CSS_SF_MIN || sf > CSS_SF_MAX || os < 1 ||
	    (uint32_t)os > max_n || (double)os * bw != fs)
		return 0;
	size_t fl;
	uint8_t *pb = packet_bytes(payload, len, seq, (uint8_t)sf, &fl);

	if (!pb)
		return 0;
	/* Bits, and the data chirps they fill (rounded up). */
	size_t nbits = (SIG_BYTES + fl) * 8, nv = (nbits + sf - 1) / sf;
	uint32_t ret = 0, N = 1u << sf;
	uint32_t gap = (uint32_t)(CSS_GAP_US * 1e-6 * (double)fs);

	if ((double)gap + ((double)CSS_PRE + nv) * N * os > max_n)
		goto fail;
	/* Samples per chirp; both it and k stay within max_n now. */
	uint32_t L = N * (uint32_t)os, k = 0;

	for (; k < gap; k++)
		PUT(k, 0, 0);
	for (size_t c = 0; c < CSS_PRE + nv; c++) {
		unsigned int s = c < CSS_PRE ? 0 :
			gray_inv(bits_at(pb, nbits, (c - CSS_PRE) * sf, sf));
		int down = c >= CSS_UP && c < CSS_PRE;
		/*
		 * f: the instantaneous frequency in units of bw, from -0.5 up,
		 * starting s chips in. The phase, in cycles from 0 at the
		 * chirp's start, is the running sum of f / os; acc keeps that
		 * sum times os, which stays exact (dyadic) when os is a power
		 * of 2. Only the fraction of a cycle goes to cosf and sinf.
		 */
		double acc = 0;

		for (uint32_t i = 0; i < L; i++, k++) {
			double f = (double)((s * (uint32_t)os + i) % L) / L -
				   0.5;

			acc += down ? -f : f;
			double cyc = acc / os;
			float a = (float)(TWO_PI * (cyc - floor(cyc)));

			PUT(k, iround(amp * cosf(a)), iround(amp * sinf(a)));
		}
	}
	ret = k;
fail:
	free(pb);
	return ret;
}

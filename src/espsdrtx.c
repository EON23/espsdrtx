/*
 * espsdrtx: arbitrary I/Q transmit through an ESP32's Wi-Fi PHY. The API is
 * in espsdrtx.h; everything chip-specific is behind hardware/port.h.
 *
 * Wi-Fi runs in NULL mode only to power the PHY; the port then tunes it and
 * drives the PHY's DAC replay engine, which plays sample words from a RAM
 * bank to the transmitter.
 *
 * Two ways to play
 * ----------------
 *   - Loop (espsdrtx_start): the engine plays words 0..n-1 over and over, in
 *     hardware, at 40 or 80 MS/s. The CPU is free meanwhile, but may not
 *     write the bank: stores into a playing loop longer than 4 words get
 *     lost or land in the wrong place (seen on the C3).
 *   - Stream (espsdrtx_*_stream): the engine plays a one-word loop at
 *     80 MS/s, and the CPU rewrites that word every `period` CPU cycles, so
 *     the sample rate is the CPU clock / period (on the 160 MHz C3, each
 *     value is held for period / 2 DAC samples), set by how fast the CPU can
 *     compute samples.
 */
#include "espsdrtx.h"

#include <math.h>
#include <stdlib.h>

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "hal/mwdt_ll.h"
#include "hardware/port.h"
#include "nvs_flash.h"
#include "soc/timer_group_struct.h"

/*
 * Frequencies, kHz. The ISM band is always allowed. Out of band (opt-in),
 * only the range where the chip's PLL locks (HW_LOCK_*_KHZ), less 2690 to
 * 2700 MHz, a radio astronomy band where all emissions are prohibited.
 */
#define ISM_LO_KHZ		2400000u
#define ISM_HI_KHZ		2483500u
#define RADIO_ASTRO_LO_KHZ	2690000u
#define RADIO_ASTRO_HI_KHZ	2700000u

/*
 * Streams. A sample takes at least about 40 CPU cycles to compute and
 * store; the upper bound only keeps the arithmetic below in range.
 */
#define PERIOD_MIN	40u
#define PERIOD_MAX	1000000u
#define WDT_FEED_US	10000u		/* feed the watchdogs every 10 ms */

/*
 * The carrier phase is a 32-bit accumulator: PHASE_TURN (2^32) is one full
 * turn, so it wraps for free. Its top LUT_BITS bits index a LUT_LEN-entry
 * table of the carrier, one entry per 1/1024 of a turn.
 */
#define PHASE_TURN	4294967296.0	/* 2^32 */
#define LUT_BITS	10
#define LUT_LEN		(1u << LUT_BITS)
#define LUT_SHIFT	(32 - LUT_BITS)	/* phase >> LUT_SHIFT: table index */

/*
 * Audio bytes are unsigned 8-bit: AUDIO_ZERO is silence, so m = byte - 128
 * runs -128..127 and m / AUDIO_ZERO is the signal from -1 to just under 1.
 */
#define AUDIO_ZERO	128
#define AUDIO_LEVELS	256

/*
 * AM's carrier table is Q14 fixed point: Q14_ONE stands for 1.0. A product
 * is rounded back by adding half (Q14_HALF) and shifting Q14_BITS right.
 */
#define Q14_BITS	14
#define Q14_ONE		(1 << Q14_BITS)
#define Q14_HALF	(1 << (Q14_BITS - 1))

#define TWO_PI		6.283185307179586

static volatile uint32_t *const bank = HW_BANK;
static bool started, tx_on;
static bool out_of_band;	/* espsdrtx_allow_out_of_band */
static int dc_i, dc_q;		/* espsdrtx_set_dc */

/* ---------------------------------------------------------- sample words */

static int clip(int v)
{
	if (v < -ESPSDRTX_MAX_WORD)
		return -ESPSDRTX_MAX_WORD;
	if (v > ESPSDRTX_MAX_WORD)
		return ESPSDRTX_MAX_WORD;
	return v;
}

/*
 * A sample's I and Q fields of the word, the DC correction added, clipped.
 * A word is field_i(i) | field_q(q). The mask keeps the low bits of the
 * two's complement value, which is how the DAC reads a negative number.
 */
static uint32_t field_i(int i)
{
	return (uint32_t)clip(i + dc_i) & HW_WORD_FIELD;
}

static uint32_t field_q(int q)
{
	return ((uint32_t)clip(q + dc_q) & HW_WORD_FIELD) << HW_WORD_Q_SHIFT;
}

/*
 * The larger DC correction: a stream's amplitude plus this must fit the
 * word, so no sample clips.
 */
static int dc_max(void)
{
	return abs(dc_i) > abs(dc_q) ? abs(dc_i) : abs(dc_q);
}

/* ------------------------------------------------------ setup and tuning */

static bool khz_ok(uint32_t khz)
{
	if (khz >= ISM_LO_KHZ && khz <= ISM_HI_KHZ)
		return true;
	return out_of_band && khz >= HW_LOCK_LO_KHZ &&
	       khz <= HW_LOCK_HI_KHZ &&
	       !(khz >= RADIO_ASTRO_LO_KHZ && khz <= RADIO_ASTRO_HI_KHZ);
}

void espsdrtx_allow_out_of_band(bool on)
{
	out_of_band = on;
}

esp_err_t espsdrtx_begin(uint32_t khz)
{
	if (started)
		return ESP_ERR_INVALID_STATE;
	/* Checked before Wi-Fi is up, so a bad frequency changes nothing. */
	if (!khz_ok(khz))
		return ESP_ERR_INVALID_ARG;
	esp_err_t e = hw_check_ram();

	if (e != ESP_OK)
		return e;
	/* Wi-Fi needs NVS; recover it the way the ESP-IDF examples do. */
	e = nvs_flash_init();
	if (e == ESP_ERR_NVS_NO_FREE_PAGES ||
	    e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		e = nvs_flash_erase();
		if (e == ESP_OK)
			e = nvs_flash_init();
	}
	if (e != ESP_OK)
		return e;
	/* INVALID_STATE: the default event loop exists already. */
	e = esp_event_loop_create_default();
	if (e != ESP_OK && e != ESP_ERR_INVALID_STATE)
		return e;
	/* Wi-Fi in NULL mode: powers the PHY without joining or sending. */
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	e = esp_wifi_init(&cfg);
	if (e != ESP_OK)
		return e;
	e = esp_wifi_set_storage(WIFI_STORAGE_RAM);
	if (e != ESP_OK)
		return e;
	e = esp_wifi_set_mode(WIFI_MODE_NULL);
	if (e != ESP_OK)
		return e;
	e = esp_wifi_start();
	if (e != ESP_OK)
		return e;
	e = esp_wifi_set_ps(WIFI_PS_NONE);
	if (e != ESP_OK)
		return e;
	e = esp_wifi_set_promiscuous(true);
	if (e != ESP_OK)
		return e;
	e = esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
	if (e != ESP_OK)
		return e;
	started = true;
	return espsdrtx_tune(khz, 0);
}

esp_err_t espsdrtx_tune(uint32_t khz, unsigned int filter)
{
	if (!started)
		return ESP_ERR_INVALID_STATE;
	if (!khz_ok(khz) || filter > HW_FILTER_MAX)
		return ESP_ERR_INVALID_ARG;
	espsdrtx_stop();
	hw_tune(khz, filter);
	return ESP_OK;
}

/* -------------------------------------------------------------- the loop */

esp_err_t espsdrtx_write(uint32_t k, int i, int q)
{
	if (tx_on)
		return ESP_ERR_INVALID_STATE;	/* the engine owns the bank */
	if (k >= ESPSDRTX_MAX_SAMPLES)
		return ESP_ERR_INVALID_ARG;
	bank[k] = field_i(i) | field_q(q);
	return ESP_OK;
}

esp_err_t espsdrtx_set_dc(int i, int q)
{
	if (i < -ESPSDRTX_MAX_WORD || i > ESPSDRTX_MAX_WORD ||
	    q < -ESPSDRTX_MAX_WORD || q > ESPSDRTX_MAX_WORD)
		return ESP_ERR_INVALID_ARG;
	dc_i = i;
	dc_q = q;
	return ESP_OK;
}

esp_err_t espsdrtx_load(const int16_t *iq, uint32_t n)
{
	if (tx_on)
		return ESP_ERR_INVALID_STATE;
	if (!iq || n > ESPSDRTX_MAX_SAMPLES)
		return ESP_ERR_INVALID_ARG;
	/* iq holds I, Q pairs: sample k is iq[2k], iq[2k + 1]. */
	for (uint32_t k = 0; k < n; k++)
		espsdrtx_write(k, iq[2 * k], iq[2 * k + 1]);
	return ESP_OK;
}

esp_err_t espsdrtx_start(uint32_t n, espsdrtx_rate_t rate)
{
	if (!started || tx_on)
		return ESP_ERR_INVALID_STATE;
	if (n < 1 || n > ESPSDRTX_MAX_SAMPLES ||
	    (rate != ESPSDRTX_40MSPS && rate != ESPSDRTX_80MSPS))
		return ESP_ERR_INVALID_ARG;
	hw_tx_take();
	hw_loop_run(n, rate);
	tx_on = true;
	return ESP_OK;
}

void espsdrtx_stop(void)
{
	if (!tx_on)
		return;
	hw_tx_off();
	hw_release();
	hw_to_rx();
	tx_on = false;
}

bool espsdrtx_running(void)
{
	return tx_on;
}

int espsdrtx_put(void *ctx, uint32_t k, int i, int q)
{
	(void)ctx;
	return espsdrtx_write(k, i, q) != ESP_OK;
}

/* --------------------------------------------------------------- streams
 *
 * Every stream has the same shape:
 *   1. check the arguments and precompute tables (the C3 has no FPU, so
 *      the loop itself does only integer arithmetic and table lookups);
 *   2. stream_on: interrupts off, a one-word loop running;
 *   3. the stream's loop: per sample, wait for its slot on the cycle counter
 *      (SLOT), store the word, step the modulator, and every `div` samples
 *      (every symbol, for QPSK) take the next byte from the caller's source;
 *   4. stream_off: TX off, interrupts back on, front end to receive.
 *
 * The loops are IRAM_ATTR so no flash cache miss stalls them, and
 * NOINLINE_ATTR because, inlined into a caller in flash, IRAM_ATTR would not
 * keep them in RAM.
 */

struct stream {
	uint32_t period;	/* CPU cycles per sample */
	uint32_t feed_every;	/* samples between watchdog feeds */
	espsdrtx_src_t src;
	void *ctx;
	espsdrtx_stream_stats_t st;
};

static esp_err_t stream_check(uint32_t period, espsdrtx_src_t src)
{
	if (!started || tx_on)
		return ESP_ERR_INVALID_STATE;
	if (period < PERIOD_MIN || period > PERIOD_MAX || !src)
		return ESP_ERR_INVALID_ARG;
	return ESP_OK;
}

/* Samples per second: CPU cycles per second over cycles per sample. */
static double stream_fs(uint32_t period)
{
	return esp_rom_get_cpu_ticks_per_us() * 1e6 / period;
}

static void stream_on(struct stream *s)
{
	/*
	 * The watchdogs are fed by interrupts, which stay off: the loop feeds
	 * them every WDT_FEED_US, which is this many samples (+ 1 so it is
	 * never 0).
	 */
	s->feed_every =
		esp_rom_get_cpu_ticks_per_us() * WDT_FEED_US / s->period + 1;
	portDISABLE_INTERRUPTS();
	bank[0] = 0;
	hw_tx_take();
	hw_stream_run();
}

static void stream_off(void)
{
	hw_tx_off();
	bank[0] = 0;
	hw_release();
	portENABLE_INTERRUPTS();
	hw_to_rx();
}

/* Both timer groups' watchdogs; each is write-protected between feeds. */
static IRAM_ATTR NOINLINE_ATTR void feed_wdts(void)
{
	mwdt_ll_write_protect_disable(&TIMERG0);
	mwdt_ll_feed(&TIMERG0);
	mwdt_ll_write_protect_enable(&TIMERG0);
	mwdt_ll_write_protect_disable(&TIMERG1);
	mwdt_ll_feed(&TIMERG1);
	mwdt_ll_write_protect_enable(&TIMERG1);
}

/*
 * One sample: busy-wait until the cycle counter reaches `target`, store the
 * word, then move `target` one period on. Slots are fixed in time, so a late
 * sample does not delay the next one. A sample whose wait ends a period or
 * more after its slot counts as late. now - target is compared as a signed
 * number, so the wait still works when the 32-bit counter wraps.
 * Uses the loop's locals s, target and feed.
 */
#define SLOT(word)							\
	do {								\
		uint32_t now;						\
									\
		while ((int32_t)((now = esp_cpu_get_cycle_count()) -	\
				 target) < 0) {				\
		}							\
		bank[0] = (word);					\
		if (now - target > s->st.worst)				\
			s->st.worst = now - target;			\
		if (now - target >= s->period)				\
			s->st.late++;					\
		target += s->period;					\
		s->st.samples++;					\
		if (--feed == 0) {					\
			feed = s->feed_every;				\
			feed_wdts();					\
		}							\
	} while (0)

/*
 * FM and PM. The carrier phase accumulator's top bits index lut, a table of
 * finished words, amp (cos, sin), around one turn. For audio byte b, with
 * m = b - AUDIO_ZERO:
 *   - FM: the phase steps by base + fdev m per sample;
 *   - PM: the phase steps by base, and the word is read at the phase plus
 *     pdev m.
 * The caller passes fdev = 0 for PM and pdev = 0 for FM.
 */
static IRAM_ATTR NOINLINE_ATTR void angle_loop(struct stream *s,
					       const uint32_t *lut,
					       uint32_t base, uint32_t fdev,
					       uint32_t pdev, uint32_t div)
{
	uint32_t phase = 0, poff = 0, inc = base;	/* poff: PM's offset */
	/* Samples until the next byte; the first comes after sample 0. */
	uint32_t until_byte = 1;
	uint32_t feed = s->feed_every;
	uint32_t target = esp_cpu_get_cycle_count() + s->period;

	for (;;) {
		SLOT(lut[(phase + poff) >> LUT_SHIFT]);
		phase += inc;
		if (--until_byte)
			continue;
		int b = s->src(s->ctx);

		if (b < 0)
			break;
		/*
		 * m as unsigned: the products then wrap modulo 2^32 like the
		 * phase, which gives the right step for a negative m too.
		 */
		uint32_t m = (uint32_t)((b & 0xff) - AUDIO_ZERO);

		inc = base + fdev * m;
		poff = pdev * m;
		until_byte = div;
	}
}

/*
 * AM. The same phase accumulator indexes cs, the carrier's cos and sin
 * (interleaved pairs, Q14), which are scaled by the byte's amplitude gain[b]
 * and rounded back to whole DAC words. The caller has made sure that
 * amplitude plus the DC correction fits the word, so the loop adds the
 * correction without clipping.
 */
static IRAM_ATTR NOINLINE_ATTR void am_loop(struct stream *s,
					    const int16_t *cs,
					    const uint16_t *gain,
					    uint32_t base, uint32_t div)
{
	/* The unmodulated level until the first byte. */
	uint32_t phase = 0, a = gain[AUDIO_ZERO];
	uint32_t until_byte = 1;
	int32_t di = dc_i, dq = dc_q;
	uint32_t feed = s->feed_every;
	uint32_t target = esp_cpu_get_cycle_count() + s->period;

	for (;;) {
		/* The table index, doubled: cs holds (cos, sin) pairs. */
		uint32_t k = (phase >> LUT_SHIFT) << 1;
		int32_t i = (cs[k] * (int32_t)a + Q14_HALF) >> Q14_BITS;
		int32_t q = (cs[k + 1] * (int32_t)a + Q14_HALF) >> Q14_BITS;

		SLOT(((uint32_t)(i + di) & HW_WORD_FIELD) |
		     (((uint32_t)(q + dq) & HW_WORD_FIELD) << HW_WORD_Q_SHIFT));
		phase += base;
		if (--until_byte)
			continue;
		int b = s->src(s->ctx);

		if (b < 0)
			break;
		a = gain[b & 0xff];
		until_byte = div;
	}
}

static esp_err_t angle_stream(uint32_t period, uint32_t div,
			      int32_t offset_hz, double fdev_hz,
			      double pdev_rad, int amp, espsdrtx_src_t src,
			      void *ctx, espsdrtx_stream_stats_t *stats)
{
	esp_err_t e = stream_check(period, src);

	if (e != ESP_OK)
		return e;
	double fs = stream_fs(period);

	/*
	 * amp plus the DC correction must fit the word, so no peak clips; the
	 * highest frequency must stay under half the sample rate (Nyquist);
	 * pdev_rad < 400 (under 128 pi) keeps pdev, its share per unit of m,
	 * under half a turn (2^31), within lround's 32-bit long.
	 */
	if (div < 1 || amp < 0 || amp + dc_max() > ESPSDRTX_MAX_WORD ||
	    fabs((double)offset_hz) + fabs(fdev_hz) >= fs / 2 ||
	    !(pdev_rad >= 0 && pdev_rad < 400))
		return ESP_ERR_INVALID_ARG;
	uint32_t *lut = malloc(LUT_LEN * sizeof(*lut));

	if (!lut)
		return ESP_ERR_NO_MEM;
	for (uint32_t k = 0; k < LUT_LEN; k++) {
		double ph = TWO_PI * k / LUT_LEN;

		lut[k] = field_i((int)lround(amp * cos(ph))) |
			 field_q((int)lround(amp * sin(ph)));
	}
	/*
	 * Phase steps in accumulator units: PHASE_TURN / fs per Hz, since a
	 * frequency of f Hz turns f / fs of a turn per sample. fdev and pdev
	 * are per unit of m (-128..127), so the deviation is divided by
	 * AUDIO_ZERO: m = 128 would give the full deviation. Negative steps
	 * are stored two's complement, which wraps the same.
	 */
	double unit = PHASE_TURN / fs;
	uint32_t base = (uint32_t)(int32_t)lround(offset_hz * unit);
	uint32_t fdev = (uint32_t)(int32_t)lround(fdev_hz * unit / AUDIO_ZERO);
	uint32_t pdev = (uint32_t)lround(pdev_rad * PHASE_TURN / TWO_PI /
					 AUDIO_ZERO);
	struct stream s = {.period = period, .src = src, .ctx = ctx};

	stream_on(&s);
	angle_loop(&s, lut, base, fdev, pdev, div);
	stream_off();
	free(lut);
	if (stats)
		*stats = s.st;
	return ESP_OK;
}

esp_err_t espsdrtx_fm_stream(uint32_t period, uint32_t div,
			     int32_t offset_hz, int32_t dev_hz, int amp,
			     espsdrtx_src_t src, void *ctx,
			     espsdrtx_stream_stats_t *stats)
{
	return angle_stream(period, div, offset_hz, dev_hz, 0, amp, src, ctx,
			    stats);
}

esp_err_t espsdrtx_pm_stream(uint32_t period, uint32_t div,
			     int32_t offset_hz, float dev_rad, int amp,
			     espsdrtx_src_t src, void *ctx,
			     espsdrtx_stream_stats_t *stats)
{
	return angle_stream(period, div, offset_hz, 0, dev_rad, amp, src, ctx,
			    stats);
}

esp_err_t espsdrtx_am_stream(uint32_t period, uint32_t div,
			     int32_t offset_hz, float depth, int amp,
			     espsdrtx_src_t src, void *ctx,
			     espsdrtx_stream_stats_t *stats)
{
	esp_err_t e = stream_check(period, src);

	if (e != ESP_OK)
		return e;
	if (div < 1 || amp < 0 || amp + dc_max() > ESPSDRTX_MAX_WORD ||
	    !(depth >= 0 && depth <= 1) ||
	    fabs((double)offset_hz) >= stream_fs(period) / 2)
		return ESP_ERR_INVALID_ARG;
	/* One allocation: cs (LUT_LEN cos, sin pairs), then gain. */
	int16_t *cs = malloc(2 * LUT_LEN * sizeof(*cs) +
			     AUDIO_LEVELS * sizeof(uint16_t));

	if (!cs)
		return ESP_ERR_NO_MEM;
	uint16_t *gain = (uint16_t *)(cs + 2 * LUT_LEN);

	for (uint32_t k = 0; k < LUT_LEN; k++) {
		double ph = TWO_PI * k / LUT_LEN;

		cs[2 * k] = (int16_t)lround(Q14_ONE * cos(ph));
		cs[2 * k + 1] = (int16_t)lround(Q14_ONE * sin(ph));
	}
	/*
	 * gain[b] = amp (1 + depth m) / (1 + depth), m = (b - 128) / 128: the
	 * unmodulated level amp / (1 + depth) for b = 128 (silent audio),
	 * rising to just under amp at b = 255 and falling to
	 * amp (1 - depth) / (1 + depth) at b = 0.
	 */
	for (int b = 0; b < AUDIO_LEVELS; b++)
		gain[b] = (uint16_t)lround(amp *
			(1 + depth * (b - AUDIO_ZERO) / (double)AUDIO_ZERO) /
			(1 + depth));
	/* The carrier's phase step, as in angle_stream. */
	uint32_t base = (uint32_t)(int32_t)lround(offset_hz * PHASE_TURN /
						  stream_fs(period));
	struct stream s = {.period = period, .src = src, .ctx = ctx};

	stream_on(&s);
	am_loop(&s, cs, gain, base, div);
	stream_off();
	free(cs);
	if (stats)
		*stats = s.st;
	return ESP_OK;
}

/*
 * Root-raised-cosine pulse, roll-off RRC_BETA, t in symbols (peak
 * 1 - b + 4 b / pi at t = 0). The two special cases are the formula's 0 / 0
 * points, at t = 0 and |t| = 1 / (4 b); RRC_EPS is how close counts as on
 * them.
 */
#define RRC_BETA	0.35
#define RRC_EPS		1e-9

static double rrc35(double t)
{
	const double b = RRC_BETA, pi = 3.141592653589793;

	if (fabs(t) < RRC_EPS)
		return 1 - b + 4 * b / pi;
	if (fabs(fabs(t) - 1 / (4 * b)) < RRC_EPS)
		return b / sqrt(2) * ((1 + 2 / pi) * sin(pi / (4 * b)) +
				      (1 - 2 / pi) * cos(pi / (4 * b)));
	return (sin(pi * t * (1 - b)) + 4 * b * t * cos(pi * t * (1 + b))) /
	       (pi * t * (1 - 16 * b * b * t * t));
}

#define Q_SPAN		6u	/* symbols one pulse spans (+-3) */
#define Q_HIST_MASK	((1u << Q_SPAN) - 1)	/* one bit per symbol */
#define Q_SPS_MIN	2u
#define Q_SPS_MAX	16u
#define Q_SYM_PER_BYTE	4	/* 2 bits (I and Q) per symbol */

/*
 * QPSK. With +-1 symbols, an output sample depends only on the position p in
 * the current symbol and on the last Q_SPAN symbols, so every possible I and
 * Q word is precomputed: qi[p][h] and qq[p][h] (stored flat as
 * [(p << Q_SPAN) | h]), where bit j of the history h is the symbol j symbols
 * ago (1: +1). Each symbol shifts one bit into the I history and one into
 * the Q history: bits 0 and 1 of the byte, then 2 and 3, and so on.
 */
static IRAM_ATTR NOINLINE_ATTR void q_loop(struct stream *s,
					   const uint32_t *qi,
					   const uint32_t *qq, uint32_t sps)
{
	/* The histories start as Q_SPAN symbols of -1. */
	uint32_t p = 0, hist_i = 0, hist_q = 0;
	/* The byte being sent, and its symbols still to go. */
	uint32_t byte = 0, sym_left = 0;
	uint32_t feed = s->feed_every;
	uint32_t target = esp_cpu_get_cycle_count() + s->period;

	for (;;) {
		SLOT(qi[(p << Q_SPAN) | hist_i] | qq[(p << Q_SPAN) | hist_q]);
		if (++p != sps)
			continue;
		p = 0;
		if (sym_left == 0) {
			int b = s->src(s->ctx);

			if (b < 0)
				break;
			byte = (uint32_t)b;
			sym_left = Q_SYM_PER_BYTE;
		}
		hist_i = ((hist_i << 1) | (byte & 1)) & Q_HIST_MASK;
		hist_q = ((hist_q << 1) | ((byte >> 1) & 1)) & Q_HIST_MASK;
		byte >>= 2;
		sym_left--;
	}
}

esp_err_t espsdrtx_qpsk_stream(uint32_t period, uint32_t sps, int amp,
			       espsdrtx_src_t src, void *ctx,
			       espsdrtx_stream_stats_t *stats)
{
	esp_err_t e = stream_check(period, src);

	if (e != ESP_OK)
		return e;
	if (sps < Q_SPS_MIN || sps > Q_SPS_MAX || amp < 0 ||
	    amp > ESPSDRTX_MAX_WORD)
		return ESP_ERR_INVALID_ARG;
	/* One allocation: qi, then qq, each sps << Q_SPAN words. */
	uint32_t *qi = malloc(2 * (sps << Q_SPAN) * sizeof(*qi));
	uint32_t *qq = qi + (sps << Q_SPAN);
	double tap[Q_SPS_MAX * Q_SPAN], peak = 0;

	if (!qi)
		return ESP_ERR_NO_MEM;
	/*
	 * tap[j sps + p]: the pulse of the symbol j symbols ago, at sample p of
	 * the current symbol; each pulse's centre lies Q_SPAN / 2 symbols after
	 * its start. peak is the largest sum of |taps| at any p: the worst
	 * case, every symbol adding up. Scaling by amp / peak puts that at amp;
	 * field_i and field_q then add the DC correction, clipping (unlike AM,
	 * FM and PM, amp plus the correction is not checked here).
	 */
	for (uint32_t p = 0; p < sps; p++) {
		double sum = 0;

		for (uint32_t j = 0; j < Q_SPAN; j++) {
			tap[j * sps + p] = rrc35((double)(p + j * sps) / sps -
						 Q_SPAN / 2.0);
			sum += fabs(tap[j * sps + p]);
		}
		if (sum > peak)
			peak = sum;
	}
	/* Every history h: symbol j is +1 if bit j is set, else -1. */
	for (uint32_t p = 0; p < sps; p++) {
		for (uint32_t h = 0; h < (1u << Q_SPAN); h++) {
			double v = 0;

			for (uint32_t j = 0; j < Q_SPAN; j++)
				v += ((h >> j) & 1 ? 1 : -1) * tap[j * sps + p];
			int w = (int)lround(v * amp / peak);

			qi[(p << Q_SPAN) | h] = field_i(w);
			qq[(p << Q_SPAN) | h] = field_q(w);
		}
	}
	struct stream s = {.period = period, .src = src, .ctx = ctx};

	stream_on(&s);
	q_loop(&s, qi, qq, sps);
	stream_off();
	free(qi);
	if (stats)
		*stats = s.st;
	return ESP_OK;
}

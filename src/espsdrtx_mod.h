/*
 * espsdrtx_mod: packet modulators for the espsdrtx loop. Plain C (no
 * ESP-IDF), so the same code runs on a host for tests.
 *
 * Every packet carries the same framing: SIGNAL (6 bytes: bits per symbol,
 * frame length, CRC-32[:3]) then the frame (sequence byte, payload, CRC-32
 * little-endian), bits LSB first. Each modulator writes its samples, DAC
 * words, through a sink, put(ctx, k, i, q) for k = 0..n-1. On the chip pass
 * espsdrtx_put (espsdrtx.h) and ESPSDRTX_MAX_SAMPLES, then
 * espsdrtx_start(n, rate). Every waveform loops without a jump, with a quiet
 * gap between repetitions. Building 16000 samples takes about a second on
 * the C3.
 *
 * Every modulator takes @payload, @len and @seq (the frame's sequence byte),
 * @max_n (the most samples it may write), and @put and @ctx (the sink and
 * its argument). It returns n, the samples written, or 0 if an argument is
 * out of range, the packet does not fit @max_n samples, memory runs out, or
 * @put returns nonzero (espsdrtx_put does while TX is on).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * typedef espsdrtx_put_t - A modulator's sample sink.
 * @ctx: the ctx passed to the modulator.
 * @k: sample index.
 * @i: in-phase word.
 * @q: quadrature word.
 *
 * Return: 0, or nonzero to abort the modulator.
 */
typedef int (*espsdrtx_put_t)(void *ctx, uint32_t k, int i, int q);

/**
 * espsdrtx_crc32() - zlib's CRC-32.
 * @b: bytes.
 * @n: their count.
 *
 * Return: the CRC.
 */
uint32_t espsdrtx_crc32(const uint8_t *b, size_t n);

/**
 * espsdrtx_sc_packet() - Single-carrier PSK or QAM packet.
 * @payload: data bytes.
 * @len: their count.
 * @seq: the frame's sequence byte.
 * @m: bits per symbol of the frame: 1 BPSK, 2 QPSK, 3 8-PSK, 4 16-QAM,
 *	6 64-QAM (all Gray coded).
 * @sps: samples per symbol, >= 2.
 * @rms: the nominal rms DAC word of unit-power symbols; peaks run higher,
 *	so keep them under the compression point.
 * @max_n: the most samples to write.
 * @put: the sink.
 * @ctx: passed to @put.
 *
 * Root-raised-cosine 0.35 shaped (circular, +-8 symbols): a 64-symbol QPSK
 * preamble, SIGNAL in QPSK, the frame at @m bits per symbol, 16 silent
 * symbols.
 *
 * Return: the samples written, or 0 (see the top of this file).
 */
uint32_t espsdrtx_sc_packet(const uint8_t *payload, size_t len, uint8_t seq,
			    int m, int sps, float rms, uint32_t max_n,
			    espsdrtx_put_t put, void *ctx);

/**
 * espsdrtx_ofdm_packet() - OFDM packet, 802.11a numerology at 80 MS/s.
 * @payload: data bytes.
 * @len: their count.
 * @seq: the frame's sequence byte.
 * @m: bits per data subcarrier: 1, 2, 4 or 6.
 * @rms: the nominal rms DAC word (peaks run several times higher).
 * @max_n: the most samples to write.
 * @put: the sink.
 * @ctx: passed to @put.
 *
 * 64 subcarriers at 312.5 kHz, built 4x oversampled: 2 silent symbols, two
 * LTF training symbols, SIGNAL in one BPSK symbol, then the frame, all
 * scrambled; 48 data subcarriers, 4 pilots. The frame may be at most
 * ESPSDRTX_WHITE_BITS / 8 bytes; a 16383-sample loop allows less anyway.
 *
 * Return: the samples written, or 0 (see the top of this file).
 */
uint32_t espsdrtx_ofdm_packet(const uint8_t *payload, size_t len,
			      uint8_t seq, int m, float rms, uint32_t max_n,
			      espsdrtx_put_t put, void *ctx);

/**
 * espsdrtx_fsk_packet() - Continuous-phase FSK packet.
 * @payload: data bytes.
 * @len: their count.
 * @seq: the frame's sequence byte.
 * @fs: sample rate, samples per second (@fs / @rate whole, >= 2).
 * @rate: symbols per second.
 * @h: modulation index: the deviation is level * @h * @rate / 2.
 * @bt: Gaussian filter of the frequency (GFSK), >= 0.1; 0 is rectangular
 *	(with @h 0.5: MSK).
 * @nlev: 2 (bit 1 = +f) or 4 (Gray, levels +-1, +-3).
 * @amp: DAC word amplitude (constant envelope).
 * @max_n: the most samples to write.
 * @put: the sink.
 * @ctx: passed to @put.
 *
 * 16 alternating preamble symbols, sync word 93 0B 51 DE, SIGNAL, the frame,
 * 4 tail symbols, edges ramped, behind a 20 us gap.
 *
 * Return: the samples written, or 0 (see the top of this file).
 */
uint32_t espsdrtx_fsk_packet(const uint8_t *payload, size_t len, uint8_t seq,
			     float fs, float rate, float h, float bt, int nlev,
			     float amp, uint32_t max_n, espsdrtx_put_t put,
			     void *ctx);

/**
 * espsdrtx_css_packet() - Chirp spread spectrum packet.
 * @payload: data bytes.
 * @len: their count.
 * @seq: the frame's sequence byte.
 * @fs: sample rate, samples per second (@fs / @bw whole).
 * @bw: chirp bandwidth: chirps sweep -@bw/2..@bw/2.
 * @sf: spreading factor, 2..12: 2^@sf chips per chirp, @sf bits per chirp
 *	(Gray).
 * @amp: DAC word amplitude (constant envelope).
 * @max_n: the most samples to write.
 * @put: the sink.
 * @ctx: passed to @put.
 *
 * A 10 us gap, 8 upchirps, 2 downchirps, then SIGNAL and the frame as one
 * bit stream. SIGNAL carries @sf as the bits per symbol. Tested receivers
 * decode @sf 2, 3, 4 and 6.
 *
 * Return: the samples written, or 0 (see the top of this file).
 */
uint32_t espsdrtx_css_packet(const uint8_t *payload, size_t len, uint8_t seq,
			     float fs, float bw, int sf, float amp,
			     uint32_t max_n, espsdrtx_put_t put, void *ctx);

#ifdef __cplusplus
}
#endif

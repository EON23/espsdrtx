/*
 * espsdrtx: arbitrary I/Q transmit through an ESP32's Wi-Fi PHY, using its
 * DAC replay engine.
 *
 * Load up to ESPSDRTX_MAX_SAMPLES complex samples (I and Q, signed, up to
 * +-ESPSDRTX_MAX_WORD) and the radio plays them in a loop, at 40 or 80 MS/s,
 * around a carrier in the 2.4 GHz ISM band. Or stream samples computed by
 * the CPU (FM, PM, AM, QPSK) for signals that never repeat.
 *
 * Supported chips: ESP32-C3 and ESP32-C6; an ESP32-S3 port is written but
 * not tested on hardware (src/hardware/ has one directory per chip). The
 * library uses undocumented PHY internals (found on ESP-IDF 5.4 and 5.5),
 * takes over the Wi-Fi radio, and reserves RAM for the RF block (128 KiB at
 * 0x3fca0000..0x3fcbffff on the C3, 0x40840000..0x4085ffff on the C6; 64 KiB
 * at 0x3fcd0000..0x3fcdffff on the S3).
 *
 * Not thread-safe: call espsdrtx from one task. Functions returning esp_err_t
 * return ESP_OK on success.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "hardware/target.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Loop sample rates. */
typedef enum {
	ESPSDRTX_40MSPS = 0,	/* the 80 MS/s DAC, every other sample 0 */
	ESPSDRTX_80MSPS = 1,
} espsdrtx_rate_t;

/**
 * espsdrtx_begin() - Power the PHY and tune.
 * @khz: carrier in kHz, as espsdrtx_tune takes it.
 *
 * Starts Wi-Fi in null mode, which powers the PHY without joining or
 * sending. Call once, before anything else here; normal Wi-Fi cannot be used
 * afterwards. Erases the default NVS partition if it is full or from a newer
 * format, as the ESP-IDF examples do.
 *
 * Return: ESP_ERR_INVALID_STATE if already called; ESP_ERR_INVALID_ARG if
 * @khz is out of range; ESP_ERR_NO_MEM if the program's static data reaches the
 * RAM the RF block takes; or the error of the NVS, event loop or Wi-Fi call
 * that failed.
 */
esp_err_t espsdrtx_begin(uint32_t khz);

/**
 * espsdrtx_allow_out_of_band() - Accept carriers outside the ISM band.
 * @on: true to accept them, false (the default) to refuse them.
 *
 * WARNING: transmitting outside the 2.4 GHz ISM band is illegal in most
 * places without a licence (2300 to 2400 MHz is part of an amateur band in
 * many countries, for licensed amateurs only), and the neighbouring bands
 * carry satellite, telemetry and mobile services. Use it only into a dummy
 * load or a coax link with an attenuator, or under a licence that covers the
 * frequency.
 *
 * With @on true, espsdrtx_begin and espsdrtx_tune accept the range where the
 * chip's PLL locked on the boards tested (2175000..2790000 kHz on the C3),
 * except 2690000..2700000 (radio astronomy). Call it before espsdrtx_begin
 * to start out of band. Turning it off does not move a carrier already out
 * of band: tune back into the band, or stop.
 */
void espsdrtx_allow_out_of_band(bool on);

/**
 * espsdrtx_tune() - Move the carrier.
 * @khz: carrier in kHz, 2400000..2483500 (wider with
 *	espsdrtx_allow_out_of_band).
 * @filter: the PHY's channel filter mode (on the C3, 0: 21 MHz within 2 dB;
 *	1 measured the same as 0; 2: about 35 MHz within 3 dB).
 *
 * Stops a running transmission.
 *
 * Return: ESP_ERR_INVALID_STATE before espsdrtx_begin; ESP_ERR_INVALID_ARG
 * if @khz or @filter is out of range.
 */
esp_err_t espsdrtx_tune(uint32_t khz, unsigned int filter);

/**
 * espsdrtx_write() - Set one sample of the loop.
 * @k: sample index, 0 <= @k < ESPSDRTX_MAX_SAMPLES.
 * @i: in-phase word.
 * @q: quadrature word.
 *
 * The DC correction (espsdrtx_set_dc) is added, then the sum clipped to
 * +-ESPSDRTX_MAX_WORD. The output compresses from about 256 (40 MS/s) or 128
 * (80 MS/s) on the C3.
 *
 * Return: ESP_ERR_INVALID_STATE while transmitting; ESP_ERR_INVALID_ARG if
 * @k is out of range.
 */
esp_err_t espsdrtx_write(uint32_t k, int i, int q);

/**
 * espsdrtx_load() - Set the first samples of the loop.
 * @iq: interleaved I, Q pairs: sample k is @iq[2k], @iq[2k + 1].
 * @n: samples, at most ESPSDRTX_MAX_SAMPLES.
 *
 * Writes samples 0..@n-1 as espsdrtx_write does.
 *
 * Return: ESP_ERR_INVALID_STATE while transmitting; ESP_ERR_INVALID_ARG if
 * @iq is NULL or @n is too large.
 */
esp_err_t espsdrtx_load(const int16_t *iq, uint32_t n);

/**
 * espsdrtx_set_dc() - Set the DC correction.
 * @i: added to every I word written from now on.
 * @q: added to every Q word.
 *
 * The carrier leaks through (on the C3, -27 to -37 dBc for a tone of
 * amplitude 100 at 80 MS/s, about -44 dBc with the correction); the offset
 * that cancels it differs per board and rate, so find it with a receiver
 * (minimise the line at the carrier) and set it after espsdrtx_begin. It
 * takes from the +-ESPSDRTX_MAX_WORD range: a sample plus the offset beyond
 * it clips. Rewrite the samples after changing it.
 *
 * Return: ESP_ERR_INVALID_ARG if @i or @q is beyond +-ESPSDRTX_MAX_WORD.
 */
esp_err_t espsdrtx_set_dc(int i, int q);

/**
 * espsdrtx_start() - Play the loop until espsdrtx_stop.
 * @n: samples in the loop (0..@n-1), 1..ESPSDRTX_MAX_SAMPLES.
 * @rate: ESPSDRTX_40MSPS or ESPSDRTX_80MSPS.
 *
 * Return: ESP_ERR_INVALID_STATE before espsdrtx_begin or while transmitting;
 * ESP_ERR_INVALID_ARG if @n or @rate is out of range.
 */
esp_err_t espsdrtx_start(uint32_t n, espsdrtx_rate_t rate);

/**
 * espsdrtx_stop() - Stop the loop; does nothing if it is not playing.
 */
void espsdrtx_stop(void);

/**
 * espsdrtx_running() - Whether the loop is playing.
 *
 * Return: true between espsdrtx_start and espsdrtx_stop.
 */
bool espsdrtx_running(void);

/**
 * espsdrtx_put() - espsdrtx_write as a modulator sink (espsdrtx_mod.h).
 * @ctx: unused; pass NULL.
 * @k: sample index.
 * @i: in-phase word.
 * @q: quadrature word.
 *
 * Pass it to the modulators.
 *
 * Return: 0, or nonzero (the modulator then returns 0) if the write fails,
 * e.g. while transmitting.
 */
int espsdrtx_put(void *ctx, uint32_t k, int i, int q);

/*
 * Continuous streams
 * ------------------
 * The loop only repeats what it holds, so a stream instead rewrites a
 * one-word loop from the CPU, one sample every `period` CPU cycles (CPU
 * clock / period samples per second: 160 MHz on the C3). Each word is held
 * until the next, so images sit at multiples of that rate.
 *
 * A stream call blocks for the whole stream with interrupts off (it feeds
 * the watchdogs itself): no other task, timer or interrupt runs, and
 * peripherals must be polled. The data comes from a source, called inside
 * the stream between two samples, so it must take well under `period`
 * cycles; keep it and what it reads in RAM (IRAM_ATTR, DRAM_ATTR) to avoid
 * flash cache misses. On the C3, period 72 and up kept every sample on time
 * in testing (the loop body takes about 40 cycles). Samples are DAC words
 * around the tuned carrier, the DC correction added.
 *
 * The analog streams (FM, PM, AM) take 8-bit unsigned audio, 128 being
 * silence: m = (byte - 128) / 128, from -1 to 127/128, one byte per `div`
 * samples, on a carrier offset_hz from the tuned frequency (under half the
 * sample rate), amplitude amp. amp plus the larger DC correction may not
 * exceed ESPSDRTX_MAX_WORD.
 *
 * Every stream returns ESP_ERR_INVALID_STATE before espsdrtx_begin or while
 * the loop plays; ESP_ERR_INVALID_ARG if period is outside 40..1000000, src
 * is NULL, or another argument is out of the range given; ESP_ERR_NO_MEM if
 * its tables cannot be allocated.
 */

/**
 * typedef espsdrtx_src_t - A stream's data source.
 * @ctx: the ctx passed to the stream call.
 *
 * Return: the next byte, 0..255, or a negative value to end the stream.
 */
typedef int (*espsdrtx_src_t)(void *ctx);

/**
 * struct espsdrtx_stream_stats - How a stream kept time.
 * @samples: words written.
 * @late: words written a period or more after their slot.
 * @worst: the largest lateness, CPU cycles.
 */
typedef struct espsdrtx_stream_stats {
	uint32_t samples;
	uint32_t late;
	uint32_t worst;
} espsdrtx_stream_stats_t;

/**
 * espsdrtx_fm_stream() - Frequency-modulate audio.
 * @period: CPU cycles per sample.
 * @div: samples per audio byte, >= 1.
 * @offset_hz: carrier offset from the tuned frequency.
 * @dev_hz: deviation: the frequency is @offset_hz + m @dev_hz;
 *	|@offset_hz| + |@dev_hz| must stay under half the sample rate.
 * @amp: amplitude, DAC words.
 * @src: the audio source.
 * @ctx: passed to @src.
 * @stats: filled in at the end; may be NULL.
 *
 * Return: as for every stream, above.
 */
esp_err_t espsdrtx_fm_stream(uint32_t period, uint32_t div,
			     int32_t offset_hz, int32_t dev_hz, int amp,
			     espsdrtx_src_t src, void *ctx,
			     espsdrtx_stream_stats_t *stats);

/**
 * espsdrtx_pm_stream() - Phase-modulate audio.
 * @period: CPU cycles per sample.
 * @div: samples per audio byte, >= 1.
 * @offset_hz: carrier offset from the tuned frequency.
 * @dev_rad: deviation: the phase is m @dev_rad radians, 0 <= @dev_rad < 400.
 * @amp: amplitude, DAC words.
 * @src: the audio source.
 * @ctx: passed to @src.
 * @stats: filled in at the end; may be NULL.
 *
 * Each audio byte steps the phase, so keep the audio rate well above the
 * audio band.
 *
 * Return: as for every stream, above.
 */
esp_err_t espsdrtx_pm_stream(uint32_t period, uint32_t div,
			     int32_t offset_hz, float dev_rad, int amp,
			     espsdrtx_src_t src, void *ctx,
			     espsdrtx_stream_stats_t *stats);

/**
 * espsdrtx_am_stream() - Amplitude-modulate audio.
 * @period: CPU cycles per sample.
 * @div: samples per audio byte, >= 1.
 * @offset_hz: carrier offset from the tuned frequency.
 * @depth: modulation depth, 0..1.
 * @amp: amplitude: the envelope is @amp (1 + @depth m) / (1 + @depth), so
 *	@amp is about the peak.
 * @src: the audio source.
 * @ctx: passed to @src.
 * @stats: filled in at the end; may be NULL.
 *
 * Return: as for every stream, above.
 */
esp_err_t espsdrtx_am_stream(uint32_t period, uint32_t div,
			     int32_t offset_hz, float depth, int amp,
			     espsdrtx_src_t src, void *ctx,
			     espsdrtx_stream_stats_t *stats);

/**
 * espsdrtx_qpsk_stream() - QPSK-modulate bytes.
 * @period: CPU cycles per sample.
 * @sps: samples per symbol, 2..16.
 * @amp: the largest I or Q word, at most ESPSDRTX_MAX_WORD. Unlike the
 *	analog streams, @amp plus the DC correction is not checked: the sum
 *	clips.
 * @src: the byte source.
 * @ctx: passed to @src.
 * @stats: filled in at the end; may be NULL.
 *
 * Two bits per symbol, LSB first (I then Q, bit 1 = positive),
 * root-raised-cosine 0.35 pulses over +-3 symbols. The filter starts from
 * six symbols of (-1, -1), and the pulse lags 3 symbols: send 1 more byte
 * than the data to play out its tail.
 *
 * Return: as for every stream, above.
 */
esp_err_t espsdrtx_qpsk_stream(uint32_t period, uint32_t sps, int amp,
			       espsdrtx_src_t src, void *ctx,
			       espsdrtx_stream_stats_t *stats);

#ifdef __cplusplus
}
#endif

/*
 * EspSdrTx: the Arduino (and C++) face of espsdrtx.h: the same calls, as one
 * object. Each method is the espsdrtx_ function of the same name, documented
 * in espsdrtx.h. (Not EspSdrTx.h: case-insensitive file systems would merge
 * it with espsdrtx.h.)
 */
#pragma once

#include "espsdrtx.h"
#include "espsdrtx_mod.h"

class EspSdrTx {
public:
	static constexpr uint32_t MAX_SAMPLES = ESPSDRTX_MAX_SAMPLES;
	static constexpr int MAX_WORD = ESPSDRTX_MAX_WORD;
	static constexpr espsdrtx_rate_t RATE_40M = ESPSDRTX_40MSPS;
	static constexpr espsdrtx_rate_t RATE_80M = ESPSDRTX_80MSPS;

	/* Read the warning in espsdrtx.h first. */
	void allowOutOfBand(bool on = true)
	{
		espsdrtx_allow_out_of_band(on);
	}

	esp_err_t begin(uint32_t khz)
	{
		return espsdrtx_begin(khz);
	}

	esp_err_t tune(uint32_t khz, unsigned int filter = 0)
	{
		return espsdrtx_tune(khz, filter);
	}

	esp_err_t write(uint32_t k, int i, int q)
	{
		return espsdrtx_write(k, i, q);
	}

	esp_err_t setDc(int i, int q)
	{
		return espsdrtx_set_dc(i, q);
	}

	esp_err_t load(const int16_t *iq, uint32_t n)
	{
		return espsdrtx_load(iq, n);
	}

	esp_err_t start(uint32_t n, espsdrtx_rate_t rate = RATE_80M)
	{
		return espsdrtx_start(n, rate);
	}

	void stop()
	{
		espsdrtx_stop();
	}

	bool running()
	{
		return espsdrtx_running();
	}

	esp_err_t fmStream(uint32_t period, uint32_t div, int32_t offsetHz,
			   int32_t devHz, int amp, espsdrtx_src_t src,
			   void *ctx = nullptr,
			   espsdrtx_stream_stats_t *stats = nullptr)
	{
		return espsdrtx_fm_stream(period, div, offsetHz, devHz, amp,
					  src, ctx, stats);
	}

	esp_err_t pmStream(uint32_t period, uint32_t div, int32_t offsetHz,
			   float devRad, int amp, espsdrtx_src_t src,
			   void *ctx = nullptr,
			   espsdrtx_stream_stats_t *stats = nullptr)
	{
		return espsdrtx_pm_stream(period, div, offsetHz, devRad, amp,
					  src, ctx, stats);
	}

	esp_err_t amStream(uint32_t period, uint32_t div, int32_t offsetHz,
			   float depth, int amp, espsdrtx_src_t src,
			   void *ctx = nullptr,
			   espsdrtx_stream_stats_t *stats = nullptr)
	{
		return espsdrtx_am_stream(period, div, offsetHz, depth, amp,
					  src, ctx, stats);
	}

	esp_err_t qpskStream(uint32_t period, uint32_t sps, int amp,
			     espsdrtx_src_t src, void *ctx = nullptr,
			     espsdrtx_stream_stats_t *stats = nullptr)
	{
		return espsdrtx_qpsk_stream(period, sps, amp, src, ctx, stats);
	}
};

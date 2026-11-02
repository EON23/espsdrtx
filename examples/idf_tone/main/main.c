/*
 * The Arduino Tone example in plain ESP-IDF: 1 MHz above 2442 MHz, 5 s on,
 * 5 s off.
 */
#include <math.h>
#include <stdio.h>

#include "espsdrtx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define KHZ	2442000u
/* 200 us at 80 MS/s: CYCLES whole cycles of 1 MHz, so the loop is seamless. */
#define N	16000u
#define CYCLES	200u
#define AMP	100
#define ON_MS	5000
#define OFF_MS	5000

void app_main(void)
{
	ESP_ERROR_CHECK(espsdrtx_begin(KHZ));
	for (uint32_t k = 0; k < N; k++) {
		/* Wrapped first (mod N): cosf of a large angle is slow. */
		float ph = 2 * (float)M_PI * (CYCLES * k % N) / N;

		espsdrtx_write(k, (int)lroundf(AMP * cosf(ph)),
			       (int)lroundf(AMP * sinf(ph)));
	}
	for (;;) {
		printf("tx on\n");
		ESP_ERROR_CHECK(espsdrtx_start(N, ESPSDRTX_80MSPS));
		vTaskDelay(pdMS_TO_TICKS(ON_MS));
		espsdrtx_stop();
		printf("tx off\n");
		vTaskDelay(pdMS_TO_TICKS(OFF_MS));
	}
}

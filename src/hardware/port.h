/*
 * The hardware port: what the chip-independent core (espsdrtx.c) needs from
 * the chip's radio. Private to the library.
 *
 * Each chip has a directory hardware/<chip>/ with:
 *   - target.h: the public limits (ESPSDRTX_MAX_SAMPLES, ESPSDRTX_MAX_WORD);
 *   - port.h: the HW_* macros below;
 *   - port.c: the hw_* functions below, compiled only for that chip;
 *   - README.md: what is known about the chip, and the boards tested.
 * hardware/README.md says how to add one.
 *
 * The model every port follows: the radio plays sample words from a RAM bank
 * in a loop of n words, HW_BANK[0..n-1], at 40 or 80 MS/s. A word holds I and
 * Q as two's complement fields HW_WORD_FIELD wide, I in the low bits and Q
 * shifted up by HW_WORD_Q_SHIFT. While the loop plays, the CPU may write only
 * a one-word loop (streams).
 *
 * Macros each port.h defines:
 *   HW_BANK		volatile uint32_t *: the loop's first word
 *   HW_WORD_FIELD	mask of one field, e.g. 0x3ffu for 10 bits
 *   HW_WORD_Q_SHIFT	bit where the Q field starts
 *   HW_LOCK_LO_KHZ	lowest carrier, kHz, where the PLL locks
 *   HW_LOCK_HI_KHZ	highest
 *   HW_FILTER_MAX	largest filter mode espsdrtx_tune accepts
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "espsdrtx.h"

#if CONFIG_IDF_TARGET_ESP32C3
#include "esp32c3/port.h"
#endif

/**
 * hw_check_ram() - Check that static data stays clear of the RF bank.
 *
 * Called by espsdrtx_begin before anything else changes.
 *
 * Return: ESP_OK, or ESP_ERR_NO_MEM if static data reaches the RAM the radio
 * takes over.
 */
esp_err_t hw_check_ram(void);

/**
 * hw_tune() - Move the carrier.
 * @khz: carrier in kHz, already range-checked by the core.
 * @filter: channel filter mode, 0..HW_FILTER_MAX.
 *
 * Runs with Wi-Fi started and nothing transmitting, and leaves the front end
 * in receive.
 */
void hw_tune(uint32_t khz, unsigned int filter);

/**
 * hw_tx_take() - Force the transmitter on and give the RF bank to the radio.
 *
 * Every store to the bank or the engine after this call lands after the
 * handover. The engine is not started yet.
 */
void hw_tx_take(void);

/**
 * hw_loop_run() - Start playing HW_BANK[0..n-1] in a loop.
 * @n: words, 1..ESPSDRTX_MAX_SAMPLES.
 * @rate: ESPSDRTX_40MSPS or ESPSDRTX_80MSPS.
 *
 * Call after hw_tx_take. Plays until hw_release.
 */
void hw_loop_run(uint32_t n, espsdrtx_rate_t rate);

/**
 * hw_stream_run() - Start a one-word loop at 80 MS/s for a stream.
 *
 * Call after hw_tx_take. The output follows every later store to HW_BANK[0]
 * until hw_release.
 */
void hw_stream_run(void);

/**
 * hw_tx_off() - Turn the transmitter off (the engine may still run).
 */
void hw_tx_off(void);

/**
 * hw_release() - Stop the engine and give the RF bank back to the CPU.
 *
 * Call after hw_tx_off.
 */
void hw_release(void);

/**
 * hw_to_rx() - Put the front end back into receive, its state between
 * transmissions.
 *
 * Call after hw_release, with interrupts on.
 */
void hw_to_rx(void);

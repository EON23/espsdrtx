/*
 * ESP32-C3: the limits the public API exposes (through hardware/target.h).
 */
#pragma once

/* The replay engine's word count is 14 bits wide; 0 plays nothing. */
#define ESPSDRTX_MAX_SAMPLES	16383u

/*
 * I and Q are 10-bit two's complement (-512..511); the library keeps to
 * +-511 so both signs reach the same level.
 */
#define ESPSDRTX_MAX_WORD	511

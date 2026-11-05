/*
 * ESP32-S3: the limits the public API exposes (through hardware/target.h).
 */
#pragma once

/*
 * The replay engine's word count is 14 bits wide on the C3; the S3 is
 * assumed the same (not tested). 0 plays nothing.
 */
#define ESPSDRTX_MAX_SAMPLES	16383u

/*
 * I and Q are 10-bit two's complement (-512..511), as the S3's RX dump
 * writes them; the library keeps to +-511 so both signs reach the same
 * level.
 */
#define ESPSDRTX_MAX_WORD	511

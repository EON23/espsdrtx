/*
 * Fixed bit sequences of the espsdrtx PHYs (espsdrtx_tables.c).
 * Bit k of t: (t[k >> 3] >> (k & 7)) & 1.
 */
#pragma once

#include <stdint.h>

#define ESPSDRTX_WHITE_BITS	16384

extern const uint8_t espsdrtx_preamble_bits[16];
extern const uint8_t espsdrtx_ltf_bits[7];
extern const uint8_t espsdrtx_pilot_bits[32];
extern const uint8_t espsdrtx_white_bits[2048];

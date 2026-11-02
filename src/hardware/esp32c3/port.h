/*
 * ESP32-C3: the HW_* macros of hardware/port.h. The registers behind them
 * are described in port.c.
 */
#pragma once

#include <stdint.h>

/* Where the engine reads its samples: the upper half of the RF block. */
#define HW_BANK		((volatile uint32_t *)0x3fcb0000u)

/*
 * A sample word: I in bits 9:0, Q in bits 19:10, each a 10-bit two's
 * complement field.
 */
#define HW_WORD_FIELD	0x3ffu
#define HW_WORD_Q_SHIFT	10

/*
 * The range where the PLL was seen to lock on the board tested; outside it
 * the carrier sticks near an edge and wanders.
 */
#define HW_LOCK_LO_KHZ	2175000u
#define HW_LOCK_HI_KHZ	2790000u

/*
 * libphy's channel filter modes: 0 is 21 MHz wide within 2 dB, 1 measured
 * the same as 0, 2 is about 35 MHz within 3 dB.
 */
#define HW_FILTER_MAX	2u

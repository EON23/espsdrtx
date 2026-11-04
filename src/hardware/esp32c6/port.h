/*
 * ESP32-C6: the HW_* macros of hardware/port.h. The registers behind them
 * are described in port.c.
 */
#pragma once

#include <stdint.h>

/* Where the engine reads its samples: SRAM block 2. */
#define HW_BANK		((volatile uint32_t *)0x40840000u)

/*
 * Where a stream's words go: the front end's TX DC register (FE_TXDC),
 * which drives the DAC while TX is forced on. The CPU cannot store into
 * the bank while the engine owns it, so streams do not use the engine.
 */
#define HW_STREAM_WORD	((volatile uint32_t *)0x600a0444u)

/*
 * A sample word: I in bits 9:0, Q in bits 19:10, each a 10-bit two's
 * complement field, in the bank and in FE_TXDC alike.
 */
#define HW_WORD_FIELD	0x3ffu
#define HW_WORD_Q_SHIFT	10

/*
 * Only the ISM band has been tried on the C6, so out of band adds nothing
 * yet: espsdrtx_allow_out_of_band() leaves the range at 2400 to 2483.5 MHz.
 */
#define HW_LOCK_LO_KHZ	2400000u
#define HW_LOCK_HI_KHZ	2483500u

/*
 * libphy's channel filter modes, accepted as on the C3 so sketches carry
 * over; on the C6 modes 0 to 3 measured the same, flat to +-10 MHz.
 */
#define HW_FILTER_MAX	2u

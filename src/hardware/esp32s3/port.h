/*
 * ESP32-S3: the HW_* macros of hardware/port.h. The registers behind them,
 * and which of them are confirmed, are described in port.c.
 *
 * Not tested on hardware yet.
 */
#pragma once

#include <stdint.h>

/*
 * Where the engine reads its samples: the 64 KiB bank at 0x3fcd0000, the
 * one Espressif's own RX dump uses on the S3.
 */
#define HW_BANK		((volatile uint32_t *)0x3fcd0000u)

/*
 * Streams rewrite a one-word loop in the bank, as on the C3, whose PHY the
 * S3's matches. If the S3's engine shuts the CPU out of its bank, as the
 * C6's does, HW_STREAM_WORD has to name a front-end register instead.
 */

/*
 * A sample word: I in bits 9:0, Q in bits 19:10, each a 10-bit two's
 * complement field (the layout the S3's RX dump writes).
 */
#define HW_WORD_FIELD	0x3ffu
#define HW_WORD_Q_SHIFT	10

/*
 * The PLL's lock range is not measured on the S3, so out of band adds
 * nothing yet: espsdrtx_allow_out_of_band() leaves the range at 2400 to
 * 2483.5 MHz.
 */
#define HW_LOCK_LO_KHZ	2400000u
#define HW_LOCK_HI_KHZ	2483500u

/*
 * libphy's channel filter modes, accepted as on the C3 (same libphy calls);
 * their widths on the S3 are not measured.
 */
#define HW_FILTER_MAX	2u

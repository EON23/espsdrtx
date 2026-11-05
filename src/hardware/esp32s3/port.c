/*
 * ESP32-S3 port: the hw_* functions of hardware/port.h.
 *
 * NOT TESTED ON HARDWARE. Written from what the C3 port does and what is
 * known about the S3; see README.md for how to bring it up.
 *
 * The S3's Wi-Fi PHY is the C3's generation: its libphy exports the same
 * functions the C3 port calls, and the RX dump engine (the replay engine's
 * receive twin) sits at the same address on both. So this port is the C3's
 * with the S3's addresses:
 *
 *   - Samples: one 32-bit word each in the 64 KiB bank at 0x3fcd0000
 *     (HW_BANK; the word format is in port.h).
 *   - SRAM usage register, SENSITIVE_INTERNAL_SRAM_USAGE_3_REG
 *     (0x600c101c), field MAC_DUMP_USAGE (bits 3:0): one bit per 64 KiB
 *     bank, bit b handing 0x3fcb0000 + b * 64 KiB to the RF block. Bit 2
 *     hands it 0x3fcd0000. Confirmed: the register and field are in
 *     ESP-IDF's sensitive_reg.h for the S3 (the C3's twin is 0x600c1020),
 *     and ESPARGOS esp-sdr's S3 receiver captures through bits 0 to 2 this
 *     way. Only that bank is reserved from the heap below.
 *   - Engine control register (0x60033d64): word count in bits 13:0, rate in
 *     bit 15 (set: 80 MS/s, clear: 40 MS/s), TRIGGER in bit 19 (held: loop)
 *     and RUN in bit 31. NOT CONFIRMED: inferred. On the C3 the RX dump
 *     control is 0x60033d5c and the replay control 8 bytes above it; on the
 *     C6 the same pair is 0x600a9004 and 0x600a900c. The S3's dump control
 *     is 0x60033d5c, as on the C3 (ESPARGOS esp-sdr).
 *   - libphy's "force TX on" (g_phyFuns[47]) for the engine's output to
 *     reach the antenna. Confirmed from libphy: phy_txtone_start calls
 *     g_phyFuns[47] with 0 then 1 on the S3 (Arduino-ESP32 3.2.0 / IDF 5.4),
 *     where the C3's calls g_phyFuns[50], the C3 port's TX force.
 *
 * Tuning goes through the same libphy channel calls as on the C3 (the S3's
 * libphy and ROM export them under the same names); whenever nothing is
 * transmitting, the front end is put back into receive (rf_to_rx).
 *
 * The S3 has two cores. The core masks interrupts only on the core that
 * calls it, so the other core keeps running Wi-Fi and the application
 * meanwhile (see README.md).
 */
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32S3

#include "../port.h"

#include "heap_memory_layout.h"

/* The RF block takes the 64 KiB bank 0x3fcd0000; keep the heap out of it. */
SOC_RESERVE_MEMORY_REGION(0x3fcd0000, 0x3fce0000, espsdrtx_rf_bank);

/* Start of the reserved bank; static data must end below it. */
#define RF_BLOCK	0x3fcd0000u

/* Engine control register (inferred, see above). */
#define DAC_CTRL	0x60033d64u
#define DAC_COUNT	0x3fffu		/* bits 13:0: words per pass */
#define DAC_80MSPS	(1u << 15)	/* clear: 40 MS/s */
#define DAC_TRIG	(1u << 19)	/* held: repeat until RUN clears */
#define DAC_RUN		(1u << 31)

/*
 * SRAM usage register (SENSITIVE_INTERNAL_SRAM_USAGE_3_REG): bits 3:0,
 * MAC_DUMP_USAGE, one bit per 64 KiB bank from 0x3fcb0000.
 */
#define SRAM_USAGE	0x600c101cu
#define USAGE_MASK	15u
#define USAGE_RF	(1u << 2)	/* bank 2, 0x3fcd0000 */

/* Index of libphy's "force TX on" function in its g_phyFuns table. */
#define PHY_FN_TXFORCE	47

/*
 * Wi-Fi channel ch (1 to 13) is centred on CH0_MHZ + CH_STEP_MHZ * ch
 * (2412, 2417, ..., 2472 MHz). Channel "0" is only the formula's origin.
 */
#define CH0_MHZ		2407
#define CH_STEP_MHZ	5
#define CH_FIRST	1
#define CH_LAST		13

/* libphy and ROM functions, undocumented; the same as the C3's. */
extern void stop_tx_tone(unsigned int stop);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned int on);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned int on);
extern void set_chanfreq(unsigned int mhz, unsigned int filter);
extern void phy_set_freq(unsigned int mhz, int khz);
extern void txcal_work_mode(void);
extern void **g_phyFuns;
extern int _heap_start;		/* linker symbol: the end of static data */

/* The usage register before TX took the bank, restored after. */
static uint32_t usage0;

static inline uint32_t rd(uint32_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void wr(uint32_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

/*
 * Xtensa: memw makes earlier stores land before later loads and stores;
 * the read-back waits until the usage register has taken the new value.
 */
static void sync_usage(void)
{
	__asm__ volatile("memw" ::: "memory");
	(void)rd(SRAM_USAGE);
}

/* libphy's TX force. */
static void phy_txforce(int on)
{
	((void (*)(int))g_phyFuns[PHY_FN_TXFORCE])(on);
}

/* Front end to receive, the state the PHY sits in between transmissions. */
static void rf_to_rx(void)
{
	stop_tx_tone(1);
	rom_pbus_workmode();
	rom_pbus_xpd_tx_off();
	rom_pbus_xpd_rx_on(1);
	rom_set_rxclk_en(1);
}

esp_err_t hw_check_ram(void)
{
	/*
	 * Static data in the bank would be overwritten by the RF block. On the
	 * S3, DRAM data is placed after the IRAM code's alias, so this covers
	 * both.
	 */
	if ((uintptr_t)&_heap_start > RF_BLOCK)
		return ESP_ERR_NO_MEM;
	return ESP_OK;
}

void hw_tune(uint32_t khz, unsigned int filter)
{
	/*
	 * As on the C3: the nearest Wi-Fi channel sets up the PHY and its
	 * filter, then phy_set_freq moves the LO to khz (whole MHz and the kHz
	 * left over) if that is not the channel's centre.
	 */
	int ch = ((int)khz - CH0_MHZ * 1000 + CH_STEP_MHZ * 1000 / 2) /
		 (CH_STEP_MHZ * 1000);

	if (ch < CH_FIRST)
		ch = CH_FIRST;
	else if (ch > CH_LAST)
		ch = CH_LAST;
	set_chanfreq(CH0_MHZ + CH_STEP_MHZ * ch, filter);
	if (khz != (CH0_MHZ + CH_STEP_MHZ * (uint32_t)ch) * 1000u)
		phy_set_freq(khz / 1000, (int)(khz % 1000));
	rf_to_rx();
}

void hw_tx_take(void)
{
	usage0 = rd(SRAM_USAGE);
	phy_txforce(1);
	wr(SRAM_USAGE, (usage0 & ~USAGE_MASK) | USAGE_RF);
	sync_usage();
}

void hw_loop_run(uint32_t n, espsdrtx_rate_t rate)
{
	/*
	 * As on the C3: count and rate, then RUN, then TRIGGER held: loop until
	 * stopped. (A one-word loop needs the opposite order; see
	 * hw_stream_run.)
	 */
	uint32_t ctrl = (n & DAC_COUNT) |
			(rate == ESPSDRTX_80MSPS ? DAC_80MSPS : 0);

	wr(DAC_CTRL, 0);
	wr(DAC_CTRL, ctrl);
	wr(DAC_CTRL, ctrl | DAC_RUN);
	wr(DAC_CTRL, ctrl | DAC_RUN | DAC_TRIG);
}

void hw_stream_run(void)
{
	/*
	 * As on the C3: TRIGGER set before RUN, so the one-word loop's output
	 * follows every store to HW_BANK[0].
	 */
	wr(DAC_CTRL, 0);
	wr(DAC_CTRL, 1u | DAC_80MSPS | DAC_TRIG);
	wr(DAC_CTRL, 1u | DAC_80MSPS | DAC_TRIG | DAC_RUN);
}

void hw_tx_off(void)
{
	rom_pbus_xpd_tx_off();
	phy_txforce(0);
}

void hw_release(void)
{
	wr(DAC_CTRL, 0);
	wr(SRAM_USAGE, usage0);
	sync_usage();
}

void hw_to_rx(void)
{
	txcal_work_mode();
	rf_to_rx();
}

#endif /* CONFIG_IDF_TARGET_ESP32S3 */

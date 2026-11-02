/*
 * ESP32-C3 port: the hw_* functions of hardware/port.h.
 *
 * The C3's Wi-Fi PHY has a DAC replay engine: a test feature that plays
 * sample words from RAM to the transmitter in place of the modem's output.
 * Espressif does not document it; the registers below were found by
 * experiment (ESP32-DATV, which sends DVB-S from a C3, drives the same
 * engine).
 *
 *   - Samples: one 32-bit word each in the RF bank at 0x3fcb0000 (HW_BANK;
 *     the word format is in port.h).
 *   - SRAM owner register (0x600c1020, bits 3:0): hands the 128 KiB block
 *     0x3fca0000..0x3fcbffff to the RF block (bit 1), which then plays from
 *     its upper 64 KiB half (bit 3) at 0x3fcb0000. The whole block is
 *     reserved from the heap below.
 *   - Engine control register (0x60033d64): word count in bits 13:0, rate in
 *     bit 15 (set: 80 MS/s, clear: 40 MS/s), TRIGGER in bit 19 (held: loop)
 *     and RUN in bit 31.
 *   - libphy has to force TX on (g_phyFuns[50]) for the engine's output to
 *     reach the antenna.
 *
 * Tuning goes through libphy's channel calls; whenever nothing is
 * transmitting, the front end is put back into receive (rf_to_rx).
 */
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C3

#include "../port.h"

#include "heap_memory_layout.h"

/*
 * Handing the bank to the RF block takes the whole 128 KiB
 * 0x3fca0000..0x3fcbffff, so keep the heap out of it.
 */
SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, espsdrtx_rf_bank);

/* Start of the reserved block; static data must end below it. */
#define RF_BLOCK	0x3fca0000u

/* Engine control register. */
#define DAC_CTRL	0x60033d64u
#define DAC_COUNT	0x3fffu		/* bits 13:0: words per pass */
#define DAC_80MSPS	(1u << 15)	/* clear: 40 MS/s */
#define DAC_TRIG	(1u << 19)	/* held: repeat until RUN clears */
#define DAC_RUN		(1u << 31)

/* SRAM owner register: bits 3:0 say who owns the 128 KiB block. */
#define SRAM_OWNER	0x600c1020u
#define OWNER_MASK	15u
#define OWNER_RF	(2u | 8u)	/* RF block, playing the upper half */

/* Index of libphy's "force TX on" function in its g_phyFuns table. */
#define PHY_FN_TXFORCE	50

/*
 * Wi-Fi channel ch (1 to 13) is centred on CH0_MHZ + CH_STEP_MHZ * ch
 * (2412, 2417, ..., 2472 MHz). Channel "0" is only the formula's origin.
 */
#define CH0_MHZ		2407
#define CH_STEP_MHZ	5
#define CH_FIRST	1
#define CH_LAST		13

/* libphy and ROM functions, undocumented. */
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

/* The owner register before TX took the block, restored after. */
static uint32_t owner0;

static inline uint32_t rd(uint32_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void wr(uint32_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
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
	/* Static data in the block would be overwritten by the RF block. */
	if ((uintptr_t)&_heap_start > RF_BLOCK)
		return ESP_ERR_NO_MEM;
	return ESP_OK;
}

void hw_tune(uint32_t khz, unsigned int filter)
{
	/*
	 * The nearest Wi-Fi channel sets up the PHY and its filter: the
	 * distance from the channel origin in channel steps, rounded (half a
	 * step added before the division), clamped to 1..13. Then, if khz is
	 * not that channel's centre, phy_set_freq moves the LO to it, given as
	 * whole MHz and the kHz left over. Out of band, the channel is 1 or 13
	 * and phy_set_freq does all the moving.
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
	/* The fence makes the owner change land before any later store. */
	owner0 = rd(SRAM_OWNER);
	phy_txforce(1);
	wr(SRAM_OWNER, (owner0 & ~OWNER_MASK) | OWNER_RF);
	__asm__ volatile("fence rw,rw" ::: "memory");
}

void hw_loop_run(uint32_t n, espsdrtx_rate_t rate)
{
	/*
	 * Count and rate, then RUN, then TRIGGER held: loop until stopped.
	 * (A one-word loop needs the opposite order; see hw_stream_run.)
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
	 * TRIGGER set before RUN: started RUN first, a one-word loop plays
	 * nothing; started this way, the output follows every store to
	 * HW_BANK[0].
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
	wr(SRAM_OWNER, owner0);
	__asm__ volatile("fence rw,rw" ::: "memory");
}

void hw_to_rx(void)
{
	txcal_work_mode();
	rf_to_rx();
}

#endif /* CONFIG_IDF_TARGET_ESP32C3 */

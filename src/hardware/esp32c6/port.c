/*
 * ESP32-C6 port: the hw_* functions of hardware/port.h.
 *
 * The C6's modem has the same kind of DAC replay engine as the C3, at other
 * addresses: it plays sample words from an SRAM block to the transmitter in
 * place of the modem's output. Espressif does not document it; the
 * registers below come from cyber-murmel's esp-sdr-zephyr
 * (doc/undocumented-registers-esp32c6.md) and were checked on the bench
 * with a HackRF (see README.md).
 *
 *   - Samples: one 32-bit word each in SRAM block 2 at 0x40840000 (HW_BANK;
 *     the word format is in port.h).
 *   - SRAM usage register (0x60095004, bits 11:8, one-hot): bit 10 hands
 *     block 2, all 128 KiB of 0x40840000..0x4085ffff, to the engine. While
 *     the engine owns it the CPU's stores are dropped and its loads return
 *     other data, so the whole block is reserved from the heap below.
 *   - Engine control register (0x600a900c): word count in bits 13:0, rate
 *     in bit 15 (set: 80 MS/s, clear: 40 MS/s), LOOP in bit 19 (replay
 *     until RUN clears) and RUN in bit 31.
 *   - The modem clocks in MODEM_SYSCON (0x600a9804, 0x600a9814, 0x600a980c)
 *     forced on: without them the engine runs but nothing reaches the
 *     antenna.
 *   - libphy's force_txon_mode() forces the transmitter on (pbus TX on alone
 *     does not); its third argument is the power index, 0 the strongest,
 *     about 1 dB weaker per step.
 *   - The DAC source select (0x600a0414, bits 1:0) at 0, which the engine
 *     plays through.
 *
 * Streams do not use the engine: the CPU cannot store into a bank the
 * engine owns. They write the front end's TX DC register (FE_TXDC,
 * 0x600a0444, the same I/Q word format), which with the transmitter forced
 * on drives the DAC directly (HW_STREAM_WORD in port.h).
 *
 * ESP-IDF's PHY PLL tracking (phy_track_pll, a 1 s esp_timer) releases the
 * TX force within about a second, which silences a loop. A loop pauses it
 * and hw_to_rx starts it again. Streams run with interrupts off, so the
 * timer cannot fire during one.
 *
 * Tuning goes through libphy's channel calls, as on the C3; whenever
 * nothing is transmitting, the front end is put back into receive
 * (rf_to_rx).
 */
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32C6

#include <stdbool.h>

#include "../port.h"

#include "heap_memory_layout.h"

/* The engine takes the whole of SRAM block 2; keep the heap out of it. */
SOC_RESERVE_MEMORY_REGION(0x40840000, 0x40860000, espsdrtx_rf_bank);

/* Start of the reserved block; static data must end below it. */
#define RF_BLOCK	0x40840000u

/* Engine control register. */
#define DAC_CTRL	0x600a900cu
#define DAC_COUNT	0x3fffu		/* bits 13:0: words per pass */
#define DAC_80MSPS	(1u << 15)	/* clear: 40 MS/s */
#define DAC_LOOP	(1u << 19)	/* replay until RUN clears */
#define DAC_RUN		(1u << 31)

/* SRAM usage register: bits 11:8 one-hot, which block the engine owns. */
#define SRAM_USAGE	0x60095004u
#define USAGE_MASK	(0xfu << 8)
#define USAGE_RF	(1u << 10)	/* block 2, 0x40840000 */

/* Modem clock enables, forced on while a loop plays. */
#define MODEM_CLK_CONF		0x600a9804u
#define MODEM_CLK_CONF1		0x600a9814u
#define MODEM_CLK_CONF1_ON	0x7ffffu	/* bits 18:0 */
#define MODEM_CLK_POWER_ST	0x600a980cu

/* Front end: DAC source select and the TX DC word. */
#define FE_DAC_SRC	0x600a0414u
#define DAC_SRC_MASK	3u
#define FE_TXDC		0x600a0444u

/* force_txon_mode's power index: 0 is the strongest. */
#define TX_POWER_INDEX	0

/*
 * Wi-Fi channel ch (1 to 13) is centred on CH0_MHZ + CH_STEP_MHZ * ch
 * (2412, 2417, ..., 2472 MHz). Channel "0" is only the formula's origin.
 */
#define CH0_MHZ		2407
#define CH_STEP_MHZ	5
#define CH_FIRST	1
#define CH_LAST		13

/* libphy and ROM functions, undocumented. */
extern void ram_stop_tx_tone(unsigned int stop);
extern void pbus_workmode(void);
extern void ram_pbus_xpd_rx_on(unsigned int on);
extern void rom_pbus_xpd_rx_on(unsigned int on);
extern void rom_pbus_xpd_tx_on(unsigned int on);
extern void rom_pbus_xpd_tx_off(void);
extern void set_rxclk_en(unsigned int on);
extern void set_chanfreq(unsigned int mhz, unsigned int filter);
extern void phy_set_freq(unsigned int mhz, int khz);
extern void txcal_work_mode(void);
extern void force_txon_mode(unsigned int on, unsigned int bt,
			    unsigned int power_index);
/*
 * ESP-IDF esp_phy: start and stop the PLL tracking timer. Without it
 * (CONFIG_ESP_PHY_DISABLE_PLL_TRACK) there is nothing to pause.
 */
#if CONFIG_ESP_PHY_DISABLE_PLL_TRACK
static void phy_track_pll_init(void) {}
static void phy_track_pll_deinit(void) {}
#else
extern void phy_track_pll_init(void);
extern void phy_track_pll_deinit(void);
#endif
extern int _heap_start;		/* linker symbol: the end of static data */

/* Registers saved before TX took them, restored after. */
static uint32_t usage0, dac_src0;
/*
 * FE_TXDC as the PHY left it, saved by hw_tune: by hw_tx_take a stream has
 * already written it.
 */
static uint32_t txdc0;
static uint32_t clk0, clk1, clkp;
static bool clk_forced;		/* hw_loop_run forced the modem clocks */
static bool track_paused;	/* hw_loop_run stopped PLL tracking */

static inline uint32_t rd(uint32_t a)
{
	return *(volatile uint32_t *)a;
}

static inline void wr(uint32_t a, uint32_t v)
{
	*(volatile uint32_t *)a = v;
}

/* RISC-V: make earlier stores land before later ones, then read back. */
static void sync_usage(void)
{
	__asm__ volatile("fence rw,rw" ::: "memory");
	(void)rd(SRAM_USAGE);
}

/* Front end to receive, the state the PHY sits in between transmissions. */
static void rf_to_rx(void)
{
	ram_stop_tx_tone(1);
	pbus_workmode();
	rom_pbus_xpd_tx_off();
	ram_pbus_xpd_rx_on(1);
	set_rxclk_en(1);
}

esp_err_t hw_check_ram(void)
{
	/* Static data in the block would be lost to the engine. */
	if ((uintptr_t)&_heap_start > RF_BLOCK)
		return ESP_ERR_NO_MEM;
	return ESP_OK;
}

void hw_tune(uint32_t khz, unsigned int filter)
{
	/*
	 * As on the C3: the nearest Wi-Fi channel sets up the PHY, then
	 * phy_set_freq moves the LO to khz if that is not the channel's
	 * centre. The filter modes measured the same on the C6.
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
	txdc0 = rd(FE_TXDC);
}

void hw_tx_take(void)
{
	dac_src0 = rd(FE_DAC_SRC);
	pbus_workmode();
	rom_pbus_xpd_rx_on(0);
	rom_pbus_xpd_tx_on(1);
	force_txon_mode(1, 0, TX_POWER_INDEX);
	wr(FE_DAC_SRC, dac_src0 & ~DAC_SRC_MASK);
	usage0 = rd(SRAM_USAGE);
	wr(SRAM_USAGE, (usage0 & ~USAGE_MASK) | USAGE_RF);
	sync_usage();
}

void hw_loop_run(uint32_t n, espsdrtx_rate_t rate)
{
	uint32_t ctrl = (n & DAC_COUNT) |
			(rate == ESPSDRTX_80MSPS ? DAC_80MSPS : 0);

	/*
	 * Stop PLL tracking, which would release the TX force; it may have
	 * run since hw_tx_take, so force TX on again after.
	 */
	if (!track_paused) {
		phy_track_pll_deinit();
		track_paused = true;
	}
	force_txon_mode(1, 0, TX_POWER_INDEX);
	clk0 = rd(MODEM_CLK_CONF);
	clk1 = rd(MODEM_CLK_CONF1);
	clkp = rd(MODEM_CLK_POWER_ST);
	wr(MODEM_CLK_CONF, 0xffffffffu);
	wr(MODEM_CLK_CONF1, clk1 | MODEM_CLK_CONF1_ON);
	wr(MODEM_CLK_POWER_ST, 0xffffffffu);
	clk_forced = true;
	/* Count and rate, then RUN and LOOP together: loop until stopped. */
	wr(DAC_CTRL, 0);
	wr(DAC_CTRL, ctrl);
	wr(DAC_CTRL, ctrl | DAC_RUN | DAC_LOOP);
}

void hw_stream_run(void)
{
	/*
	 * Nothing to start: with TX forced on, FE_TXDC (HW_STREAM_WORD)
	 * drives the DAC. The core has already written it 0.
	 */
}

void hw_tx_off(void)
{
	rom_pbus_xpd_tx_off();
	force_txon_mode(0, 0, TX_POWER_INDEX);
}

void hw_release(void)
{
	wr(DAC_CTRL, 0);
	wr(FE_TXDC, txdc0);
	wr(FE_DAC_SRC, dac_src0);
	wr(SRAM_USAGE, usage0);
	sync_usage();
	if (clk_forced) {
		wr(MODEM_CLK_CONF, clk0);
		wr(MODEM_CLK_CONF1, clk1);
		wr(MODEM_CLK_POWER_ST, clkp);
		clk_forced = false;
	}
}

void hw_to_rx(void)
{
	txcal_work_mode();
	rf_to_rx();
	if (track_paused) {
		phy_track_pll_init();
		track_paused = false;
	}
}

#endif /* CONFIG_IDF_TARGET_ESP32C6 */

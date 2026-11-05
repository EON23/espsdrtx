/*
 * A continuous FM stream: a 1 kHz audio tone, 75 kHz deviation, on a carrier
 * 250 kHz above 2442 MHz, 10 s on, 5 s off. The audio comes from a function
 * the stream calls for each audio sample (31.25 kS/s: 2 MS/s output, one
 * audio sample per 64), generated on the chip.
 */
#include <espsdrtx.hpp>

EspSdrTx tx;

/*
 * CPU cycles per output sample, for 2 MS/s: 80 at 160 MHz (C3, C6), 120 at
 * 240 MHz (S3).
 */
static const uint32_t PERIOD = F_CPU / 2000000;
/* Output samples per audio sample: 2 MS/s / 64 = 31.25 kS/s audio. */
static const uint32_t DIV = 64;
static const uint32_t AUDIO_RATE = 31250;	/* audio samples per second */
static const uint32_t KHZ = 2442000;		/* the tuned frequency */
static const int32_t OFFSET_HZ = 250000;	/* carrier above KHZ */
static const int32_t DEV_HZ = 75000;		/* peak frequency deviation */
static const int AMP = 50;			/* peak DAC word */
static const uint32_t ON_S = 10, OFF_S = 5;	/* seconds on and off */

/*
 * One period of 250 Hz at 31.25 kS/s (125 samples), 8-bit unsigned audio
 * (128 is silence, so 128 + 127 sin keeps it within 1..255). Stepping
 * through it 4 entries at a time plays 1 kHz.
 */
#define SINE_LEN	125
#define SINE_STEP	4
static DRAM_ATTR uint8_t sine[SINE_LEN];
static DRAM_ATTR uint32_t pos, left;

/* Called inside the stream with interrupts off: keep it short and in RAM. */
static IRAM_ATTR int audio(void *)
{
	if (left == 0)
		return -1;	/* ends the stream */
	left--;
	return sine[(pos++ * SINE_STEP) % SINE_LEN];
}

void setup()
{
	Serial.begin(115200);
	delay(1000);

	for (int k = 0; k < SINE_LEN; k++)
		sine[k] = 128 + lroundf(127 * sinf(2 * (float)M_PI * k /
						   SINE_LEN));
	if (tx.begin(KHZ) != ESP_OK) {
		Serial.println("espsdrtx begin failed");
		while (true)
			delay(1000);
	}
}

void loop()
{
	left = ON_S * AUDIO_RATE;
	pos = 0;
	Serial.println("SENDING");
	espsdrtx_stream_stats_t st = {};
	esp_err_t e = tx.fmStream(PERIOD, DIV, OFFSET_HZ, DEV_HZ, AMP,
				  audio, nullptr, &st);

	/*
	 * USB serial counts the host as gone after the interrupts-off
	 * stretch and drops output until it sees it again (5 to 15 ms).
	 */
	for (uint32_t t = millis(); !Serial && millis() - t < 1000;)
		delay(5);
	Serial.printf("FM %s: %u samples, %u late\n", esp_err_to_name(e),
		      (unsigned int)st.samples, (unsigned int)st.late);
	delay(OFF_S * 1000);
}

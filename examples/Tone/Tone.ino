/*
 * A 1 MHz tone 1 MHz above 2442 MHz, computed on the chip: 5 s on, 5 s off.
 */
#include <espsdrtx.hpp>
#include <math.h>

EspSdrTx tx;

static const uint32_t KHZ = 2400000;
/*
 * 16000 samples are 200 us at 80 MS/s, which holds CYCLES = 200 whole
 * cycles of 1 MHz, so the loop repeats without a jump.
 */
static const uint32_t N = 16000;
static const uint32_t CYCLES = 200;
/* DAC words; 80 MS/s compresses from about 128. */
static const int AMP = 100;
static const uint32_t ON_MS = 5000, OFF_MS = 5000;

void setup()
{
	Serial.begin(115200);
	esp_err_t e = tx.begin(KHZ);

	if (e != ESP_OK) {
		Serial.printf("espsdrtx begin: %s\n", esp_err_to_name(e));
		while (true)
			delay(1000);
	}
	for (uint32_t k = 0; k < N; k++) {
		/*
		 * Sample k is CYCLES k / N turns in; the whole turns are
		 * dropped (mod N) first, since cosf of a large angle takes
		 * about 0.2 ms here.
		 */
		float ph = 2 * (float)M_PI * (CYCLES * k % N) / N;

		tx.write(k, lroundf(AMP * cosf(ph)), lroundf(AMP * sinf(ph)));
	}
}

void loop()
{
	Serial.println("tx on");
	tx.start(N, EspSdrTx::RATE_80M);
	delay(ON_MS);
	tx.stop();
	Serial.println("tx off");
	delay(OFF_MS);
}

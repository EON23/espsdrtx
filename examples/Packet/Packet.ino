/*
 * A QPSK packet built on the chip, repeated: 5 Msym/s (16 samples per symbol
 * at 80 MS/s) around 2442 MHz, 5 s on, 5 s off. Each loop carries the
 * message once; a receiver that knows the preamble (the research
 * repository's packet.sc_receive) decodes it.
 */
#include <espsdrtx.hpp>

EspSdrTx tx;
uint32_t n;

static const uint32_t KHZ = 2442000;
static const int M = 2;		/* bits per symbol: QPSK */
/* Samples per symbol: 80 MS/s / 16 = 5 Msym/s. */
static const int SPS = 16;
static const float RMS = 25;	/* rms DAC word */
static const uint8_t SEQ = 0;	/* the packet's sequence number */
static const uint32_t ON_MS = 5000, OFF_MS = 5000;

void setup()
{
	const char msg[] = "hello from espsdrtx";

	Serial.begin(115200);
	if (tx.begin(KHZ) != ESP_OK) {
		Serial.println("espsdrtx begin failed");
		while (true)
			delay(1000);
	}
	/* sizeof(msg) - 1: the text without its terminating NUL. */
	n = espsdrtx_sc_packet((const uint8_t *)msg, sizeof(msg) - 1, SEQ, M,
			       SPS, RMS, EspSdrTx::MAX_SAMPLES, espsdrtx_put,
			       NULL);
	Serial.printf("packet: %u samples\n", (unsigned int)n);
}

void loop()
{
	if (!n)
		return;
	tx.start(n, EspSdrTx::RATE_80M);
	delay(ON_MS);
	tx.stop();
	delay(OFF_MS);
}

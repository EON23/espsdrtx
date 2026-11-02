# espsdrtx

Arbitrary I/Q transmit through an ESP32's Wi-Fi PHY. You write up to 16383 complex samples
(10-bit signed I and Q on the C3), and the PHY's DAC replay engine plays them in a loop at
40 or 80 MS/s around a carrier in the 2.4 GHz ISM band. It works as an Arduino library
(`EspSdrTx`) and as an ESP-IDF component (`espsdrtx`). So far it runs on the ESP32-C3 only
(see [Tested hardware](#tested-hardware)). It was called `c3tx` before.

```cpp
#include <espsdrtx.hpp>
EspSdrTx tx;

void setup() {
  tx.begin(2442000);                       // carrier in kHz
  for (uint32_t k = 0; k < 16000; k++)     // 1 MHz tone: 200 whole cycles per loop
    tx.write(k, lroundf(100 * cosf(2 * (float)M_PI * (200 * k % 16000) / 16000)),
                lroundf(100 * sinf(2 * (float)M_PI * (200 * k % 16000) / 16000)));
  tx.start(16000, EspSdrTx::RATE_80M);     // loops until tx.stop()
}
void loop() {}
```

## API (`src/espsdrtx.h`; `EspSdrTx` wraps the same calls)

Every function is documented where it is declared, in `src/espsdrtx.h` and
`src/espsdrtx_mod.h`.

| Call | |
|---|---|
| `espsdrtx_begin(khz)` | Starts Wi-Fi in null mode (this powers the PHY) and tunes. Call once. |
| `espsdrtx_allow_out_of_band(on)` | Lets `espsdrtx_begin` and `espsdrtx_tune` accept 2175000 to 2790000 kHz (but not 2690000 to 2700000, radio astronomy). Off by default. Read **Legal** below first. |
| `espsdrtx_tune(khz, filter)` | 2400000 to 2483500 kHz (wider with `espsdrtx_allow_out_of_band`). `filter` 0: 21 MHz within 2 dB. 1: measured the same as 0. 2: about 35 MHz within 3 dB. |
| `espsdrtx_write(k, i, q)` / `espsdrtx_load(iq, n)` | Writes samples into the loop (clipped to +-511). Only while stopped. |
| `espsdrtx_set_dc(i, q)` | DC correction added to every sample written afterwards (rewrite the samples after changing it), to cancel the carrier leak. Find it per board and rate with a receiver. The sample plus the offset clips at +-511. |
| `espsdrtx_start(n, rate)` / `espsdrtx_stop()` | Plays samples 0..n-1 in a loop at `ESPSDRTX_40MSPS` or `ESPSDRTX_80MSPS` / stops. |

All but `espsdrtx_stop` and `espsdrtx_running` return an `esp_err_t`.

## Packet modulators (`src/espsdrtx_mod.h`)

They build a whole packet on the chip and write it into the loop through a sink. On the
chip that sink is `espsdrtx_put`. They are plain C, so the same code also runs on a PC.

```cpp
uint32_t n = espsdrtx_sc_packet(data, len, seq, 2 /* QPSK */, 16 /* samples/symbol */, 25 /* rms */,
                            EspSdrTx::MAX_SAMPLES, espsdrtx_put, NULL);
if (n) tx.start(n, EspSdrTx::RATE_80M);       // 0: the packet does not fit the loop
```

| Call | Modulation | Tested on air |
|---|---|---|
| `espsdrtx_sc_packet` | BPSK, QPSK, 8-PSK, 16-QAM, 64-QAM (Gray), RRC 0.35 | QPSK, 8-PSK, 16-QAM at 80 MS/s |
| `espsdrtx_ofdm_packet` | OFDM, 802.11a numerology, BPSK to 64-QAM per subcarrier, scrambled; 80 MS/s only | QPSK, 16-QAM |
| `espsdrtx_fsk_packet` | 2-FSK, MSK, GFSK (BT >= 0.1), 4-GFSK | MSK, GFSK, 4-GFSK at 40 MS/s |
| `espsdrtx_css_packet` | chirp spread spectrum (LoRa-like, not LoRa-compatible) | SF6, 10 MHz, at 40 MS/s |

Every packet carries the same framing: a preamble or sync, SIGNAL (bits per symbol,
length, CRC), then the frame (a sequence byte, the payload and a CRC-32). A receiver needs
the PHY's parameters (rate, h, bandwidth, SF) and the preamble, nothing else.
- **Build time:** a 16000-sample packet takes about 0.4 s (OFDM) to 0.9 s on the
  C3, which has no FPU.
- **Not included:** receivers.
- **Busy loop:** a modulator returns 0 if the sink fails, as `espsdrtx_put` does while TX is
  on, so stop before building the next packet.

## Continuous streams (`src/espsdrtx.h`)

The loop only repeats what it holds, so for a signal that never repeats (audio, a data
stream) the CPU rewrites a one-word loop itself, one sample every `period` CPU cycles: 160 MHz
/ 80 = 2 MS/s. A callback supplies the data, a byte at a time, and ends the stream by
returning a negative value.

```cpp
static IRAM_ATTR int audio(void *ctx) { ... return next_byte; }   // or -1: the end
tx.fmStream(80 /* period */, 64 /* samples per audio byte */, 250000 /* offset Hz */,
            75000 /* deviation Hz */, 100 /* amplitude */, audio);
```

| Call | Modulation | Tested on air (2 MS/s) |
|---|---|---|
| `espsdrtx_fm_stream` | FM of 8-bit unsigned audio, one byte per `div` samples | speech at 31.25 kS/s, 75 kHz deviation; a 1 kHz tone at 5 and 75 kHz |
| `espsdrtx_pm_stream` | PM: phase deviation in radians | a 1 kHz tone at 1 and 5 rad; at 5 rad as clean as FM at the same 5 kHz |
| `espsdrtx_am_stream` | AM: depth 0 to 1, `amp` about the peak | a 1 kHz tone at depth 0.8, peak 100 |
| `espsdrtx_qpsk_stream` | QPSK of bytes (2 bits per symbol), RRC 0.35 over +-3 symbols, 2 to 16 samples per symbol | 250 and 500 ksym/s; a file in packets, bit-exact |

- **Interrupts stay off** for the whole call, which blocks: no other task, timer or
  interrupt runs (the call feeds the watchdogs). Anything the callback reads must be
  polled or already in memory.
- **The callback runs between two samples:** keep it short and in RAM (`IRAM_ATTR`, data
  `DRAM_ATTR`). With the data in RAM, every sample came on time at 2.857 MS/s (FM, PM;
  period 56), 2.353 (AM) and 2.222 (QPSK), and FM decoded cleanly at 4 MS/s, period 40, a
  few hundred samples a little late. Fed from USB-Serial-JTAG, the data tops out between
  131.6 kB/s (carried) and 138.9 kB/s (not).
- **Images:** each word is held until the next, so copies of the signal sit at multiples of
  the sample rate, damped only by the hold.
- **Arduino `Serial`** after a stream: Arduino's USB serial counts the host as gone and
  drops output until it sees it again, 5 to 15 ms later. Wait for it before printing:
  `while (!Serial) delay(5);` (with a time limit if no host may be attached).
- **AM level:** AM is the one stream whose envelope varies; keep its peak `amp` under about
  128, where the DAC at 80 MS/s starts to compress (peak 200 cut a depth of 0.8 to 0.72).
- **PM and a receiver:** the transmitter's and receiver's oscillators wander apart in phase
  (radians, mostly below 100 Hz), so a PM receiver has to track the carrier and pass only the
  audio band; FM's frequency detector is far less sensitive to that.
- **No framing:** `espsdrtx_qpsk_stream` sends the bytes as they come; for packets, put a
  preamble and the framing into the bytes yourself.

## Limits and costs

- **Chips:** ESP32-C3 only so far (see [Tested hardware](#tested-hardware)). Tested on
  ESP-IDF 5.5 and on Arduino-ESP32 3.2.0 (IDF 5.4). Other targets fail the build with a
  message.
- **Wi-Fi:** the library takes over the radio, so normal Wi-Fi cannot run alongside it.
- **NVS:** `espsdrtx_begin` erases the default NVS partition if it is full or from a newer
  format (as ESP-IDF examples do).
- **Threads:** call espsdrtx from one task; it keeps global state without locks.
- **RAM:** on the C3 it reserves 128 KiB (0x3fca0000..0x3fcbffff), which the RF block uses for the
  samples. If the program's static data reaches that region, `espsdrtx_begin` returns
  `ESP_ERR_NO_MEM`, and the ESP-IDF build fails at link time.
- **Loop:** at most 16383 samples, 205 us at 80 MS/s. Pick a length where the waveform
  wraps without a jump, for example whole cycles of a tone.
- **Level:** there is no gain control, only the sample amplitude. The output compresses
  from about 256 at 40 MS/s and 128 at 80 MS/s (C3).
- **LO leakage:** the carrier leaks through as a DC term unless `espsdrtx_set_dc` cancels it.
- **Undocumented internals:** the library relies on libphy functions Espressif does not
  document (on the C3: `g_phyFuns[50]`, `set_chanfreq`, `phy_set_freq`, ROM `pbus` calls).
  A new IDF version can break them.
- **Legal:** by default the frequency range is limited to the 2.4 GHz ISM band. Keep the power
  low, or use a coax link with an attenuator.
- **Out of band (`espsdrtx_allow_out_of_band`): not legal to radiate in most places.** Outside the
  ISM band you need a licence that covers the frequency (2300 to 2400 MHz is part of an amateur
  band in many countries, for licensed amateurs only). Otherwise transmit only into a dummy load
  or a coax link with an attenuator. The neighbouring bands carry satellite, telemetry and
  mobile services. The carrier locked from 2175 to 2790 MHz on the board tested; outside that
  range it sticks near an edge of the range and wanders, so the library refuses it.

## Tested hardware

| Chip | Board | Toolchains | Status |
|---|---|---|---|
| ESP32-C3 (QFN32, rev v0.4) | ESP32-C3 SuperMini (40 MHz crystal, 4 MB flash, chip antenna) | ESP-IDF 5.5, Arduino-ESP32 3.2.0 | All features, on air |

The numbers in this README (levels, leakage, stream rates, the tuning range) were measured
on that one board. What is known per chip, and the boards tested, is in
`src/hardware/<chip>/README.md`.

## Adding a chip

Everything chip-specific sits in `src/hardware/<chip>/`: the registers of the replay
engine, how libphy tunes, and the RAM the radio takes. The rest of the library (the loop
API, the streams and the modulators) is shared. `src/hardware/README.md` lists what a port
has to provide.

## Install

- **Arduino:** copy or symlink this directory into your `Arduino/libraries`, or zip it and use
  *Sketch > Include Library > Add .ZIP Library*. Select the board *ESP32C3 Dev Module*
  and set *USB CDC On Boot* to enabled for `Serial` over USB. Examples: `examples/Tone`, `examples/Packet`, `examples/FmTone`, `examples/AmTone`, `examples/PmTone`.
- **ESP-IDF:** keep the directory named `espsdrtx`, and put it under your project's
  `components/` or list it in `EXTRA_COMPONENT_DIRS`. Example: `examples/idf_tone`
  (`idf.py build` from there).
- **Copying out:** copy from a clean checkout (`git archive`), or delete `examples/*/build`
  first, so no build output goes with it.

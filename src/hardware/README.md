# Hardware ports

One directory per chip. The rest of the library (`espsdrtx.c`, the modulators)
never touches the radio; it goes through the port interface in `port.h`. (The
streams feed the watchdogs through ESP-IDF's HAL, which every ESP32 has.)

| Directory | Chip | Status |
|---|---|---|
| `esp32c3/` | ESP32-C3 | Supported; boards tested are in its README |
| `esp32c6/` | ESP32-C6 | Supported (Arduino); boards tested are in its README |

## Recording a board

[`record.py`](record.py) records the examples off the air from the board
you are testing, using **a second ESP32 as the receiver**, so no SDR is
needed. For each test it saves a recording, a spectrum image and the
measurements, the same set the C3 and C6 have in their `captures/`
folders, and writes a summary to paste into a pull request.

### What you need

- **The board under test**, with the espsdrtx examples (from the branch you
  are testing, if it is a new port).
- **A receiver:** any other ESP32 board running ESPARGOS's
  [ESP-SDR](https://github.com/ESPARGOS/esp-sdr) firmware. Flash it from the
  [browser installer](https://espargos.net/espsdr/app/flash.html) (Chrome or
  Edge) and pick the chip. ESP-SDR runs on the ESP32, C2, C3, C5, C6, C61, S2
  and S3; the H2 (a Bluetooth radio, 11 MHz wide at most) is too narrow for
  the packet test.
  The receiver can be the same kind of chip as the board under test, but it
  must be a different board.
- **A PC** with Python 3 and `pip install numpy matplotlib pyserial`.
  Optional: [arduino-cli](https://arduino.github.io/arduino-cli/) with the
  ESP32 core, to flash the examples for you; a C compiler (`cc`), to build
  the packet the packet test compares against (without one it uses the copy
  stored with the C3's recordings).

Plug both boards into the PC, attach both antennas, and put the boards
0.5 to 2 m apart. Closer is fine: the script lowers the receiver's gain if
it overloads.

### Running it

From the repository's top folder, with your own ports. The receiver is
`--rx-port`, the board under test is `--port`:

```sh
# Builds and flashes each example onto the board under test, then records it
python3 src/hardware/record.py --rx-port /dev/ttyACM1 \
    --chip esp32s3 --board "Waveshare ESP32-S3-Zero" \
    --fqbn esp32:esp32:esp32s3 --port /dev/ttyACM0

# Flash the examples yourself (Arduino IDE ...): the script asks for each one
python3 src/hardware/record.py --rx-port /dev/ttyACM1 \
    --chip esp32s3 --board "Waveshare ESP32-S3-Zero"

# One test, with its example already running on the board
python3 src/hardware/record.py --rx-port /dev/ttyACM1 \
    --chip esp32s3 --board "Waveshare ESP32-S3-Zero" --tests packet --yes
```

On Windows the ports are `COM3`, `COM4` and so on. `--chip` is the ESP-IDF
target name of the board under test (`esp32`, `esp32c3`, `esp32s3` ...) and
picks the output folder. `--board` names the board in the files and images.
`--note` records anything else worth knowing (antennas, distance, a cable and
attenuator instead of antennas). `python3 src/hardware/record.py --help`
lists every option.

The three tests take a few minutes in all:

| Test | Example | Receiver | What is measured |
|---|---|---|---|
| `tone` | `Tone`: 1 MHz tone above a 2442 MHz carrier, 80 MS/s loop | 2438 MHz, 40 MS/s, 8 bursts | crystal offset, carrier leak and image (dBc) |
| `fmtone` | `FmTone`: FM, 1 kHz audio, 75 kHz deviation, 250 kHz above the carrier | 2441 MHz, 16 MS/s, 48 bursts | carrier leak (dBc of the FM power), 99 % bandwidth |
| `packet` | `Packet`: QPSK packet loop, 5 Msym/s | 2438 MHz, 40 MS/s, 8 bursts | correlation and EVM against the same packet built on the PC, QPSK decisions that agree |

The receiver records bursts of about 16,000 samples (0.4 ms at 40 MS/s, 1 ms
at 16 MS/s) with gaps between them. The script keeps only bursts taken while
the example is transmitting, free of interference outside the signal's band.
It sets the receiver's gain itself, lowering it when a burst clips and
raising it when the signal is weak or not seen. It tunes the receiver so
that none of the board's lines fall on the receiver's own DC spike or its
own image (the receiver's image is a mirror of what it receives about its
centre frequency; the images mark both).

### What you get

In `src/hardware/<chip>/captures/`, with the board's name as the suffix:

- `tone_2443MHz_<board>`, `fmtone_2442.25MHz_<board>`,
  `packet_qpsk_2442MHz_<board>`: each a `.sigmf-data` recording, its
  `.sigmf-meta` and a `.png` spectrum. The metadata records the board, what
  esptool said about its chip (with `--port`), the receiver and its firmware
  version, the gain used, and the measurements.
- `packet_qpsk_reference.cs16`: the packet built on the PC (int16 I/Q at
  80 MS/s).
- `results_<board>.md` and `.json`: the measurements, with the C3 and C6
  figures for comparison.

Open a pull request with the folder's new files and paste `results_<board>.md`
into its description.

The recordings are [SigMF](https://sigmf.org): interleaved 16-bit I/Q
(`ci16_le`), one capture segment per burst. The receiver's 10-bit samples run
to ±`espsdrtx:full_scale` (512) in the metadata. The FmTone recording is
shifted to 2442.25 MHz and decimated to 2 MS/s on the PC to keep it small (its
`core:hw` says so; full scale 8192). With NumPy:

```python
import json, numpy as np
base = "src/hardware/esp32s3/captures/tone_2443MHz_waveshare_esp32_s3_zero"
meta = json.load(open(base + ".sigmf-meta"))
raw = np.fromfile(base + ".sigmf-data", dtype=np.int16)
iq = (raw[0::2] + 1j * raw[1::2]) / meta["global"]["espsdrtx:full_scale"]
starts = [c["core:sample_start"] for c in meta["captures"]] + [len(iq)]
bursts = [iq[a:b] for a, b in zip(starts, starts[1:])]
```

### Reading the numbers

- **Crystal offset** is the board's carrier against the *receiver's* crystal,
  so it is the two boards' errors combined, not the board's own. A few tens
  of kHz (10 to 20 ppm) is normal.
- **Levels** are relative: the receiver's gain index is not in dB, and the
  distance between the boards is not controlled. Compare leak and image in
  dBc, not absolute levels.
- **The receiver adds its own noise, image and spurs**, so EVM and the floor
  come out worse than with a lab SDR. The C3 and C6 recordings were made with
  a HackRF.

### When it does not work

- **"does not answer SYNC":** check `--rx-port`, and close the ESP-SDR
  browser viewer or anything else holding the port. On Linux, ModemManager
  can grab ESP32 USB ports; stop it or tell it to ignore them.
- **"no signal from the board":** open the board's serial monitor and check
  that the example started (Tone prints `tx on`, FmTone `SENDING`, Packet the
  packet's length at start-up), and that both antennas are attached.
- **Files from another SDR:** `--input file.sigmf-data --tests tone`
  measures and plots a SigMF recording made earlier, with an ESP receiver, a
  HackRF or anything else (`ci8`, `ci16_le` or `cf32_le`).

So far `record.py` has been checked against the C3 and C6 HackRF recordings
(`--input` gives the same figures as their READMEs) and against a simulated
ESP-SDR receiver, not yet with a real one. Reports are welcome.

## What a port provides

A directory `hardware/<chip>/`, named as ESP-IDF names the target
(`IDF_TARGET`, e.g. `esp32s3`), with:

- `target.h`: the public limits, `ESPSDRTX_MAX_SAMPLES` (the longest loop)
  and `ESPSDRTX_MAX_WORD` (the largest I or Q word).
- `port.h`: the `HW_*` macros listed at the top of `../port.h`: where the
  loop's words live, the word format, the range where the PLL locks, the
  filter modes.
- `port.c`: the `hw_*` functions declared in `../port.h` (RAM check, tuning,
  start and stop of the replay engine, back to receive). Wrap the whole file
  in `#if CONFIG_IDF_TARGET_<CHIP>`: Arduino compiles every `.c` under `src/`.
- `README.md`: what is known about the chip's PHY, and a table of the boards
  tested on it, with what was measured on each.
- Optionally `rf_bank_guard.ld`, a linker script the ESP-IDF build adds (for
  example to fail the link if static data reaches RAM the radio takes over).

Then add the chip to the `#if` in `target.h` and `port.h` in this directory,
to the table above, and to "Tested hardware" in the library README.

## The model a port has to fit

The core assumes the radio plays sample words from a RAM bank in a loop of n
words at 40 or 80 MS/s, and that some word the CPU can store to reaches the
DAC as soon as it is written (the streams rely on that). By default that word
is the bank's first, played as a one-word loop at 80 MS/s (the C3); a port
whose engine shuts the CPU out of the bank names another in `HW_STREAM_WORD`
(the C6 uses the front end's TX DC register). A word holds I and Q as two's
complement fields of the same width, I in the low bits. A chip whose engine
works differently needs the interface changed, not just a new port.

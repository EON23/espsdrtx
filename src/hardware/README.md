# Hardware ports

One directory per chip. The rest of the library (`espsdrtx.c`, the modulators)
never touches the radio; it goes through the port interface in `port.h`. (The
streams feed the watchdogs through ESP-IDF's HAL, which every ESP32 has.)

| Directory | Chip | Status |
|---|---|---|
| `esp32c3/` | ESP32-C3 | Supported; boards tested are in its README |
| `esp32c6/` | ESP32-C6 | Supported (Arduino); boards tested are in its README |

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

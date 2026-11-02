# ESP32-C3

The Wi-Fi PHY's DAC replay engine, found by experiment (Espressif does not
document it; ESP32-DATV drives the same engine). The registers and the libphy
calls are described at the top of `port.c`.

| | |
|---|---|
| Loop | up to 16383 words of 10-bit I and Q, at 40 or 80 MS/s |
| Sample bank | 0x3fcb0000; the whole 128 KiB 0x3fca0000..0x3fcbffff is reserved from the heap |
| CPU | 160 MHz RISC-V, no FPU: streams run at 160 MHz / period samples per second |
| Tuning | 2400 to 2483.5 MHz; out of band (opt-in) 2175 to 2790 MHz, where the PLL locked |
| Filter modes | 0: 21 MHz within 2 dB; 1: measured the same as 0; 2: about 35 MHz within 3 dB |
| Toolchains | ESP-IDF 5.5, Arduino-ESP32 3.2.0 (IDF 5.4) |

## Boards tested

| Board | Chip | Measured |
|---|---|---|
| ESP32-C3 SuperMini (40 MHz crystal, 4 MB flash, chip antenna) | QFN32, rev v0.4 | Loop playback, all packet modulators and streams on air; the tuning range; the carrier leak and its DC correction |

## Known behaviour

- Output compresses from a word of about 256 at 40 MS/s and 128 at 80 MS/s.
- The carrier leaks through at -27 to -37 dBc (a tone of amplitude 100 at
  80 MS/s), about -44 dBc with the DC correction; the correction differs per
  board and rate.
- Writing into a playing loop longer than 4 words loses stores or puts them in
  the wrong place, so the library refuses writes while the loop plays.
- On some boots the AM stream runs slightly slower than on others: at the
  shortest periods it otherwise keeps up with (48 cycles), nearly every
  sample then comes late. The same firmware after a reset can be fine. The
  cause is still being looked into; leave some margin in the period.

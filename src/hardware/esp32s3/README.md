# ESP32-S3

**Not tested on hardware.** The port builds (Arduino-ESP32 3.2.0 / IDF 5.4,
and ESP-IDF 5.5), but no S3 has run it yet. What follows is what it is based
on, how sure each part is, and what to try first when one arrives.

The S3's Wi-Fi PHY is the C3's generation. Its libphy and ROM export the
same functions the C3 port calls, under the same names, and the receive twin
of the replay engine (the RX dump) sits at the same address on both. So
`port.c` is the C3 port with the S3's addresses.

| | |
|---|---|
| Loop | up to 16383 words of 10-bit I and Q, at 40 or 80 MS/s (assumed as on the C3) |
| Sample bank | 0x3fcd0000, 64 KiB; only that bank, 0x3fcd0000..0x3fcdffff, is reserved from the heap |
| Streams | a one-word loop in the bank, as on the C3 |
| CPU | 240 MHz Xtensa LX7, two cores: streams run at 240 MHz / period samples per second |
| Tuning | 2400 to 2483.5 MHz. The lock range is not measured, so `espsdrtx_allow_out_of_band` adds nothing on the S3 |
| Filter modes | 0 to 2 accepted as on the C3; not measured |
| Toolchains | builds on Arduino-ESP32 3.2.0 (IDF 5.4; generic *ESP32S3 Dev Module*) and ESP-IDF 5.5 (`idf.py set-target esp32s3`) |

## Where each part comes from

| Part | Value on the S3 | How sure |
|---|---|---|
| SRAM usage register | `SENSITIVE_INTERNAL_SRAM_USAGE_3_REG` (0x600c101c), field `MAC_DUMP_USAGE`, bits 3:0 | **Confirmed**: in ESP-IDF's `soc/esp32s3/register/soc/sensitive_reg.h`. The C3's is the same register at 0x600c1020 |
| Its encoding | one bit per 64 KiB bank from 0x3fcb0000; bit 2 is 0x3fcd0000 | **Confirmed for capture**: ESPARGOS [esp-sdr](https://github.com/ESPARGOS/esp-sdr)'s S3 receiver uses bit 2 for single captures and bits 0 to 2 for its ring. Not tried for transmit |
| Sample bank | 0x3fcd0000 | **Likely**: on the C3 and C6 the replay engine plays from the bank the RX dump writes to. 0x3fcd0000 is the bank Espressif's own S3 dump uses (`MAC_DUMP_USAGE` = 4) |
| Word format | I in bits 9:0, Q in 19:10 | **Likely**: the RX dump's format on the S3, and the replay format on the C3 and C6 |
| TX force | `g_phyFuns[47]` | **Confirmed from libphy**: the S3's `phy_txtone_start` calls `g_phyFuns[47]` with 0 then 1, where the C3's calls `g_phyFuns[50]`, the C3 port's TX force |
| Engine control register | 0x60033d64; count in bits 13:0, 80 MS/s in bit 15, TRIGGER in 19, RUN in 31 | **Inferred, the weakest part**: on the C3 the replay control is 8 bytes above the RX dump control (0x60033d5c, 0x60033d64), on the C6 too (0x600a9004, 0x600a900c). The S3's dump control is 0x60033d5c, the same as the C3's. Neither the S3's libphy nor its ROM touches 0x60033d64 |
| Tuning, back to receive | `set_chanfreq`, `phy_set_freq`, `stop_tx_tone`, `txcal_work_mode` (libphy); `rom_pbus_workmode`, `rom_pbus_xpd_rx_on`, `rom_pbus_xpd_tx_off`, `rom_set_rxclk_en` (ROM) | **Confirmed present** with the C3's names; `phy_set_freq` takes the same (MHz, kHz offset) arguments |

## Bringing it up

Run the Tone example with a receiver on 2443 MHz and work down the list
until it plays:

1. **Nothing at all on air.** While the loop plays, print the engine
   control register from the sketch (`*(volatile uint32_t *)0x60033d64`).
   If it does not read back with RUN (bit 31) and the count set, 0x60033d64
   is the wrong register; look near the RX dump control (0x60033d5c to
   0x60033d9c) for one with the C3's layout.
2. **Engine runs, but no signal.** Check the TX force: `g_phyFuns[47]` should
   turn the transmitter on. On the C6, the pbus calls were not enough and
   `force_txon_mode()` was needed; the S3's libphy may export that too.
3. **A loop plays, then goes silent within a few seconds.** That was ESP-IDF's
   PLL tracking timer on the C6 (`phy_track_pll`, every second). The C6 port
   shows the fix: `phy_track_pll_deinit()` while a loop plays,
   `phy_track_pll_init()` in `hw_to_rx`. Both exist in the S3's ESP-IDF.
4. **Loops work but streams are silent.** Then the S3's engine shuts the CPU
   out of its bank, as the C6's does. Streams then need `HW_STREAM_WORD` set
   to a front-end register that drives the DAC (the C6 uses its TX DC
   register; the S3's equivalent is not known).
5. **Wrong frequency, or a mirrored spectrum.** Check the word format (I and
   Q swapped, or a field width other than 10 bits).

Then measure what the C3 and C6 READMEs record: the tone and its image, the
carrier leak, the filter modes, the lock range, stream rates, and the
Packet example decoded against the PC reference.

## Differences from the C3

- **Bank:** the S3's `MAC_DUMP_USAGE` gives the RF block one 64 KiB bank,
  not a 128 KiB block, so the library reserves 64 KiB, not 128.
- **TX force:** the same libphy function, at index 47 instead of 50.
- **Two cores:** streams mask interrupts only on the core that calls them.
  The other core keeps running (in Arduino, Wi-Fi and the esp_timer task
  run on core 0, `loop()` on core 1). Anything on the other core that waits
  on the streaming core, such as a flash write or an IPC call, stalls until
  the stream ends. The other core's memory traffic may also delay samples,
  so expect more late samples than on the C3.
- **Clock:** 240 MHz, so streams run at 240 MHz / period. The stream
  examples take their period from `F_CPU` (120 on the S3) to keep 2 MS/s.

## Boards tested

None yet.

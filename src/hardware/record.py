#!/usr/bin/env python3
"""Record espsdrtx's examples off the air, using a second ESP32 as receiver.

The receiver is any ESP32 running ESPARGOS's ESP-SDR firmware
(https://github.com/ESPARGOS/esp-sdr, flashed from
https://espargos.net/espsdr/app/flash.html). The board under test runs each
espsdrtx example; this script records it through the receiver and writes,
for every test, a SigMF recording (.sigmf-data + .sigmf-meta), a spectrum
image (.png) and the measurements, plus a summary to paste into a pull
request. src/hardware/README.md explains how to use it.

  python3 src/hardware/record.py --rx-port /dev/ttyACM1 \\
      --chip esp32s3 --board "My S3 board" \\
      --fqbn esp32:esp32:esp32s3 --port /dev/ttyACM0

Tests (--tests, default all three):
  tone    examples/Tone:   1 MHz tone above a 2442 MHz carrier, 80 MS/s loop
  fmtone  examples/FmTone: FM stream 250 kHz above the carrier
  packet  examples/Packet: QPSK packet loop, checked against the same packet
          built on this PC from the library's own modulator

With --fqbn and --port the script builds and flashes each example onto the
board under test (arduino-cli); without them it asks you to flash each one
and press Enter. --input analyses a SigMF recording made earlier (by an ESP
receiver, a HackRF or another SDR) instead of recording.

Needs Python 3 with NumPy, Matplotlib and pyserial; arduino-cli to flash; a
C compiler (cc) for the packet check.
"""
import argparse
import datetime
import glob
import json
import os
import re
import secrets
import shutil
import subprocess
import sys
import tempfile
import time
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))     # src/hardware
SRC = os.path.dirname(HERE)                           # src
REPO = os.path.dirname(SRC)

SEG = 2048                    # samples per segment when looking for the signal

# The examples' frequencies (examples/*.ino). Change them here if a branch
# changes the examples.
CARRIER = 2442.0e6            # every example tunes here
TONE = CARRIER + 1e6          # Tone: complex tone 1 MHz above the carrier
TONE_IMAGE = CARRIER - 1e6    # where the transmitter's image of it falls
FM = CARRIER + 250e3          # FmTone: FM centre
FM_DEV = 75e3                 # FmTone: peak deviation
PKT_HALF = 3.375e6            # Packet: half its width, 5 Msym/s RRC 0.35

# Per test: the receiver's centre (whole MHz: ESP-SDR tunes in MHz), the
# sample rates to try in order, bursts to keep, the band the signal fills,
# the example's on/off cycle, and how far over the floor counts as strong.
#   - tone and packet at 2438 MHz: every transmitter line (tone +5 MHz, leak
#     +4, image +3; the packet +0.6 to +7.4 MHz) is clear of the receiver's
#     own DC spike and of its own image (mirrored about 2438 MHz).
#   - fmtone at 2441 MHz and a low rate, for fine frequency bins.
TESTS = {
    "tone": dict(example="Tone", base="tone_2443MHz", rx_mhz=2438,
                 rates=[40e6, 80e6, 32e6, 20e6], bursts=8, cycle_s=10,
                 band=(TONE - 150e3, TONE + 150e3), target_db=25),
    "fmtone": dict(example="FmTone", base="fmtone_2442.25MHz", rx_mhz=2441,
                   rates=[16e6, 10e6, 8e6, 20e6, 40e6], bursts=48,
                   cycle_s=15, band=(FM - 250e3, FM + 250e3), target_db=20),
    "packet": dict(example="Packet", base="packet_qpsk_2442MHz", rx_mhz=2438,
                   rates=[40e6, 80e6, 20e6, 16e6], bursts=8, cycle_s=10,
                   band=(CARRIER - 3e6, CARRIER + 3e6), target_db=15),
}

# ESP-SDR's rate codes (docs/protocol.md).
RATE_CODES = {0: 80e6, 1: 40e6, 2: 20e6, 3: 10e6, 4: 8e6, 5: 4e6, 6: 16e6,
              7: 32e6, 8: 32e6 / 3, 9: 6.4e6}


def log(*a):
    print(*a, flush=True)


def die(msg):
    sys.exit("record.py: " + msg)


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


# --------------------------------------------------------- the receiver

class EspSdr:
    """An ESP32 running ESP-SDR, over its serial protocol (protocol.md)."""

    def __init__(self, port, baud):
        try:
            import serial
        except ImportError:
            die("pyserial is missing: pip install pyserial")
        self.ser = serial.Serial(port, baud, timeout=3)
        time.sleep(0.3)           # opening some ports resets the board
        self.sync()
        info = self.cmd("INFO").split()
        if len(info) < 4 or info[2] != "burst":
            die(f"{port}: not ESP-SDR burst firmware (INFO: {' '.join(info)})")
        self.family, self.max_n = info[0], int(info[3])
        self.version = self.cmd_json("VERSION?", "VERSION")
        lim = self.cmd_json("LIMITS?", "LIMITS") or {}
        self.gain_lim = lim.get("gain") or [0, 0, 1]
        self.bw = lim.get("bandwidth")
        self.rates = lim.get("rates") or [80e6, 40e6, 16e6]
        self.bits = 10 if 10 in (lim.get("bits") or [8]) else 8
        self.gain = None

    def sync(self):
        for _ in range(10):
            nonce = secrets.token_hex(4)
            self.ser.reset_input_buffer()
            self.ser.write(f"SYNC {nonce}\n".encode())
            t0 = time.time()
            while time.time() - t0 < 2:
                line = self.ser.readline().decode(errors="replace").strip()
                if line == f"SYNC {nonce}":
                    return
            time.sleep(0.5)       # still booting
        die("the receiver does not answer SYNC: is ESP-SDR flashed, and the "
            "port right? (Close the browser viewer if it is connected.)")

    def cmd(self, c):
        self.ser.write((c + "\n").encode())
        line = self.ser.readline().decode(errors="replace").strip()
        if not line:
            die(f"the receiver did not answer {c!r}")
        if line.startswith("ERR"):
            raise RuntimeError(f"{c}: {line}")
        return line

    def cmd_json(self, c, tag):
        try:
            line = self.cmd(c)
        except RuntimeError:
            return None
        return json.loads(line[len(tag):].strip()) if line.startswith(tag) \
            else None

    def setup(self, mhz, rate):
        self.cmd(f"FREQ {mhz}")
        if self.bw:
            self.cmd("BANDWIDTH 0")           # the widest analog filter
        self.code = min(RATE_CODES, key=lambda k: abs(RATE_CODES[k] - rate))

    def set_gain(self, g):
        self.cmd(f"GAIN MANUAL {g}")
        self.gain = g

    def capture(self, n):
        """One burst: complex samples, full scale 1, and raw (I, Q) ints."""
        fmt = "CAP20" if self.bits == 10 else "CAP16"
        head = self.cmd(f"{fmt} {n} {self.code}").split()
        if head[0] != "DATA":
            die("unexpected reply: " + " ".join(head))
        n, crc = int(head[1]), int(head[2], 16)
        size = -(-n * self.bits * 2 // 8)
        data = self.ser.read(size)
        if len(data) != size or zlib.crc32(data) & 0xffffffff != crc:
            self.sync()
            return None
        if self.bits == 8:
            v = np.frombuffer(data, dtype=np.int8).astype(np.int16)
            i, mq = v[0::2], v[1::2]
            full = 128
        else:
            b = np.frombuffer(data + b"\0" * (-len(data) % 5), dtype=np.uint8)
            b = b.reshape(-1, 5).astype(np.uint64)
            w = sum(b[:, k] << np.uint64(8 * k) for k in range(5))
            s = np.empty(2 * len(w), dtype=np.uint64)
            s[0::2] = w & np.uint64(0xfffff)
            s[1::2] = (w >> np.uint64(20)) & np.uint64(0xfffff)
            s = s[:n].astype(np.int32)
            i, mq = s & 0x3ff, (s >> 10) & 0x3ff
            i = np.where(i >= 512, i - 1024, i)
            mq = np.where(mq >= 512, mq - 1024, mq)
            full = 512
        q = -mq                                # ESP-SDR sends (I, -Q)
        raw = np.empty(2 * n, dtype=np.int16)
        raw[0::2], raw[1::2] = i, q
        return raw, full

    def release(self):
        try:
            self.cmd("GAIN HARDWARE")
            self.cmd("RELEASE")
        except Exception:
            pass
        self.ser.close()


# ---------------------------------------------------------- the board

def find_esptool():
    for name in ("esptool", "esptool.py"):
        if shutil.which(name):
            return [name]
    hits = sorted(glob.glob(os.path.expanduser(
        "~/.arduino15/packages/esp32/tools/esptool_py/*/esptool")))
    return [hits[-1]] if hits else None


def chip_info(port):
    """What esptool says about the board under test (best effort)."""
    tool = find_esptool()
    if not tool:
        return {}
    r = run(tool + ["--port", port, "flash_id"], timeout=60)
    if r.returncode:
        r = run(tool + ["--port", port, "flash-id"], timeout=60)
    out = r.stdout + r.stderr
    info = {}
    for key, pat in (("chip", r"Chip (?:is|type:)\s*(.+)"),
                     ("features", r"Features:\s*(.+)"),
                     ("crystal", r"Crystal (?:is|frequency:)\s*(.+)"),
                     ("flash", r"Detected flash size:\s*(.+)")):
        m = re.search(pat, out)
        if m:
            info[key] = m.group(1).strip()
    return info


def flash(args, example):
    sketch = os.path.join(REPO, "examples", example)
    build = os.path.join(tempfile.gettempdir(), "espsdrtx-record",
                         args.fqbn.replace(":", "_"), example)
    log(f"  building and flashing examples/{example} onto the board ...")
    r = run([args.arduino_cli, "compile", "--fqbn", args.fqbn,
             "--library", REPO, "--build-path", build, "-u", "-p",
             args.port, sketch], timeout=900)
    if r.returncode:
        die(f"arduino-cli failed for {example}:\n{r.stdout}\n{r.stderr}")
    time.sleep(3)             # boot, Wi-Fi start, first transmission


# ------------------------------------------------- finding the signal

def iq(raw, full):
    x = raw.astype(np.float64) / full
    return x[0::2] + 1j * x[1::2]


def burst_check(x, fs, centre, band, skip_frac=0.0):
    """Is the signal on for the whole of x, with no burst of interference?

    Splits x into SEG-sample segments and compares, in each, the power in
    `band` (absolute Hz) with the floor. Returns (start segment of the
    longest clean stretch, its length in segments, its median SNR in dB).
    """
    nseg = len(x) // SEG
    w = np.hanning(SEG)
    f = centre + np.fft.fftshift(np.fft.fftfreq(SEG, 1 / fs))
    inb = (f >= band[0]) & (f <= band[1])
    p = np.empty((nseg, SEG))
    for k in range(0, nseg, 256):
        blk = x[k * SEG:min(nseg, k + 256) * SEG].reshape(-1, SEG) * w
        p[k:k + len(blk)] = np.abs(np.fft.fftshift(np.fft.fft(blk, axis=1),
                                                   axes=1)) ** 2
    floor = np.median(p, axis=1)
    snr = 10 * np.log10(p[:, inb].sum(axis=1) / (floor * inb.sum()) + 1e-12)
    quiet = (p.mean(axis=0) < 2 * np.median(p)) & ~inb
    oob = p[:, quiet].mean(axis=1) if quiet.any() else floor
    ok = (snr > 6) & (oob < 4 * np.median(oob))
    ok[:int(nseg * skip_frac)] = False
    best = (0, 0)
    s = None
    for k, good in enumerate(list(ok) + [False]):
        if good and s is None:
            s = k
        elif not good and s is not None:
            best = max(best, (k - s, s))
            s = None
    length, start = best
    med = float(np.median(snr[start:start + length])) if length else \
        float(np.max(snr)) if nseg else -99.0
    return start, length, med


def record_bursts(rx, t, n):
    """Collect t['bursts'] clean bursts while the example transmits.

    Adjusts the receiver's manual gain until bursts neither clip nor sit
    near the floor. Returns (list of raw int16 bursts, full scale, gain).
    """
    fs = RATE_CODES[rx.code]
    centre = t["rx_mhz"] * 1e6
    gmin, gmax, gstep = rx.gain_lim
    gstep = max(1, gstep)
    move = max(gstep, (gmax - gmin) // 8 // gstep * gstep)
    if rx.gain is None:
        rx.set_gain(gmin + (gmax - gmin) // 2 // gstep * gstep)
    need = n // SEG                    # whole segments in a burst
    got, full = [], 512
    t_gain = time.time()
    t_end = time.time() + 6 * t["cycle_s"] + 30
    while len(got) < t["bursts"]:
        if time.time() > t_end:
            if got:
                log(f"  only {len(got)} bursts; keeping them")
                break
            die("no signal from the board. Is the example running (serial "
                "output), are both antennas attached, and are the boards "
                "within a metre or two of each other?")
        r = rx.capture(n)
        if r is None:
            continue
        raw, full = r
        x = iq(raw, full)
        start, length, snr = burst_check(x, fs, centre, t["band"])
        peak = int(np.max(np.abs(raw)))
        if length < need:
            # Off, or caught at an on/off edge. After a whole cycle without
            # the signal, it may be too weak to see: more gain.
            if time.time() - t_gain > t["cycle_s"] + 2 and rx.gain < gmax:
                rx.set_gain(min(gmax, rx.gain + move))
                log(f"  nothing seen; gain index {rx.gain}")
                got, t_gain = [], time.time()
            continue
        if peak >= full - 12 and rx.gain > gmin:
            rx.set_gain(max(gmin, rx.gain - move))
            log(f"  clipping; gain index {rx.gain}")
            got, t_gain = [], time.time()
            continue
        if snr < t["target_db"] and rx.gain < gmax and peak < full // 4:
            rx.set_gain(min(gmax, rx.gain + move))
            log(f"  weak ({snr:.0f} dB over the floor); gain index {rx.gain}")
            got, t_gain = [], time.time()
            continue
        got.append(raw)
        t_gain = time.time()
        log(f"  burst {len(got)}/{t['bursts']}: {snr:.0f} dB over the floor, "
            f"peak {peak} of {full}")
    return got, full, rx.gain


# ------------------------------------------------------------ analysis

def nfft_for(fs, bin_hz, n):
    """A power of two near fs / bin_hz, no longer than a burst."""
    k = 1 << int(round(np.log2(fs / bin_hz)))
    while k > n:
        k //= 2
    return k


def welch(bursts, fs, nfft):
    """Averaged power spectrum over every burst (Hann, 75 % overlap)."""
    w = np.hanning(nfft)
    acc, cnt = 0.0, 0
    for x in bursts:
        for i in range(0, len(x) - nfft + 1, nfft // 4):
            acc = acc + np.abs(np.fft.fft(x[i:i + nfft] * w)) ** 2
            cnt += 1
    p = np.fft.fftshift(acc / max(cnt, 1)) / np.sum(w ** 2) / fs
    return np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs)), p


def db(a, b):
    return 10 * np.log10((a + 1e-30) / (b + 1e-30))


def line_power(fa, p, f0, bins=2):
    k = int(np.argmin(np.abs(fa - f0)))
    return float(p[max(0, k - bins):k + bins + 1].sum())


def tone_offset(x, fs, centre):
    """Frequency of the strongest line near TONE minus TONE, Hz."""
    n = 1 << 20
    X = np.abs(np.fft.fft(x * np.hanning(len(x)), n))
    f = centre + np.fft.fftfreq(n, 1 / fs)
    k = np.where(np.abs(f - TONE) < 150e3)[0]
    i = k[np.argmax(X[k])]
    a, b, c = (np.log(X[(i + d) % n] + 1e-30) for d in (-1, 0, 1))
    d = 0.5 * (a - c) / (a - 2 * b + c) if a - 2 * b + c else 0.0
    return float(f[i] + d * fs / n - TONE)


def clear_of_dc(f, centre):
    return abs(f - centre) > 300e3


def analyse_tone(bursts, fs, centre):
    off = float(np.median([tone_offset(x, fs, centre) for x in bursts]))
    fa, p = welch(bursts, fs, nfft_for(fs, 10e3, len(bursts[0])))
    fa = fa + centre
    tone = line_power(fa, p, TONE + off)
    res = {"carrier_offset_hz": round(off),
           "tone_over_floor_db": round(db(p.max(), np.median(p)), 1)}
    for key, f in (("leak_dbc", CARRIER + off),
                   ("image_dbc", TONE_IMAGE + off)):
        if clear_of_dc(f, centre) and fa[0] < f < fa[-1]:
            res[key] = round(db(line_power(fa, p, f), tone), 1)
    return res


FM_AUDIO = 1e3               # FmTone: the audio tone, Hz


def fm_offset(bursts, fs, centre):
    """The FM carrier's offset from FM, Hz, from its instantaneous frequency.

    A burst is about one audio cycle, too short for the spectrum to centre
    on the carrier. Instead: keep only the FM band (FM +-170 kHz, which
    leaves out the carrier leak 250 kHz below), take the instantaneous
    frequency, and average it over a whole number of audio cycles, where
    the audio averages to zero.
    """
    offs = []
    for x in bursts:
        n = int(len(x) // (fs / FM_AUDIO) * (fs / FM_AUDIO))
        if n < fs / FM_AUDIO:
            continue
        X = np.fft.fft(x[:n] * np.exp(-2j * np.pi * (FM - centre) *
                                      np.arange(n) / fs))
        f = np.fft.fftfreq(n, 1 / fs)
        X[np.abs(f) > 170e3] = 0
        y = np.fft.ifft(X)
        offs.append(np.mean(np.angle(y[1:] * np.conj(y[:-1]))) * fs /
                    (2 * np.pi))
    return float(np.median(offs)) if offs else 0.0


def analyse_fm(bursts, fs, centre, off):
    fa, p = welch(bursts, fs, nfft_for(fs, 2.4e3, len(bursts[0])))
    fa = fa + centre
    if off is None:                   # no tone result: from the FM itself
        off = fm_offset(bursts, fs, centre)
    k = np.abs(fa - (FM + off)) <= 100e3
    fm = float(p[k].sum())
    res = {"carrier_offset_hz": round(off),
           "leak_dbc_of_fm": round(db(line_power(fa, p, CARRIER + off), fm),
                                   1)}
    k = np.where(np.abs(fa - (FM + off)) < 250e3)[0]
    c = np.cumsum(p[k]) / p[k].sum()
    res["occupied_bw_99_khz"] = round(
        (fa[k][np.searchsorted(c, 0.995)] - fa[k][np.searchsorted(c, 0.005)])
        / 1e3, 1)
    return res


PACKET_MAIN = r"""
#include <stdio.h>
#include <stdint.h>
#include "espsdrtx_mod.h"
static int16_t iq[2 * 16383];
static int put(void *ctx, uint32_t k, int i, int q)
{
	(void)ctx;
	if (k >= 16383)
		return 1;
	iq[2 * k] = (int16_t)i;
	iq[2 * k + 1] = (int16_t)q;
	return 0;
}
int main(void)
{
	const char msg[] = MSG;
	uint32_t n = espsdrtx_sc_packet((const uint8_t *)msg, sizeof(msg) - 1,
					SEQ, M, SPS, RMS, 16383, put, NULL);
	fwrite(iq, sizeof(int16_t), 2 * n, stdout);
	return n == 0;
}
"""


def packet_reference():
    """The Packet example's loop built on this PC: int16 I/Q, 80 MS/s.

    Reads the example's parameters from examples/Packet/Packet.ino and
    builds it with the library's own modulator (needs cc). Falls back to
    the reference stored with the C3's recordings.
    """
    ino = open(os.path.join(REPO, "examples", "Packet", "Packet.ino")).read()

    def grab(pat, dflt):
        m = re.search(pat, ino)
        return m.group(1) if m else dflt
    defs = {"MSG": grab(r'const char msg\[\]\s*=\s*("(?:[^"\\]|\\.)*")',
                        '"hello from espsdrtx"'),
            "SEQ": grab(r"SEQ\s*=\s*(\d+)", "0"),
            "M": grab(r"\bM\s*=\s*(\d+)", "2"),
            "SPS": grab(r"SPS\s*=\s*(\d+)", "16"),
            "RMS": grab(r"RMS\s*=\s*([\d.]+)", "25")}
    cc = shutil.which("cc") or shutil.which("gcc") or shutil.which("clang")
    if cc:
        with tempfile.TemporaryDirectory() as d:
            main = os.path.join(d, "ref.c")
            open(main, "w").write(PACKET_MAIN)
            exe = os.path.join(d, "ref")
            r = run([cc, "-O2", "-std=gnu99", "-I", SRC] +
                    [f"-D{k}={v}" for k, v in defs.items()] +
                    [main, os.path.join(SRC, "espsdrtx_mod.c"),
                     os.path.join(SRC, "espsdrtx_tables.c"), "-lm", "-o", exe])
            if r.returncode == 0:
                out = subprocess.run([exe], capture_output=True)
                if out.returncode == 0 and out.stdout:
                    return np.frombuffer(out.stdout, dtype=np.int16).copy()
            log("  could not build the packet on this PC:\n" + r.stderr)
    path = os.path.join(HERE, "esp32c3", "captures",
                        "packet_qpsk_reference.cs16")
    log("  using the stored reference " + os.path.relpath(path, REPO))
    return np.fromfile(path, dtype=np.int16)


def packet_one(x, fs, centre, r80, dec, want_cfo):
    """One burst against the reference: (rho, evm, agree, cfo, loops)."""
    L = len(r80) // dec
    x = x * np.exp(-2j * np.pi * (CARRIER - centre) * np.arange(len(x)) / fs)
    loops = len(x) // L - 1
    if loops < 3:
        return None
    seg0 = np.array([x[i * L:(i + 1) * L] for i in range(loops)])
    c = np.sum(seg0[1:] * np.conj(seg0[:-1]), axis=1)
    dphi = np.angle(np.sum(c))
    n = np.arange(L)
    # The loop-to-loop phase step gives the offset only modulo fs / L: try
    # each wrap (or the one nearest the tone's offset) and keep the best.
    step = fs / L
    base = dphi / (2 * np.pi) * step
    if want_cfo is None:
        cfos = [base + k * step for k in range(-4, 5)]
    else:
        cfos = [base + round((want_cfo - base) / step) * step]
    best = None
    for cfo in cfos:
        s = seg0 * np.exp(-1j * dphi * np.arange(loops))[:, None]
        s = s * np.exp(-2j * np.pi * cfo * n / fs)[None, :]
        yy = s.mean(axis=0)
        Y = np.fft.fft(yy)
        for ph in range(dec):
            rr = r80[ph::dec][:L]
            cc = np.fft.ifft(Y * np.conj(np.fft.fft(rr)))
            k = int(np.argmax(np.abs(cc)))
            rho = abs(cc[k]) / (np.linalg.norm(yy) * np.linalg.norm(rr))
            if best is None or rho > best[0]:
                best = (rho, cfo, ph, k, yy)
    rho, cfo, ph, lag, yy = best
    ref = r80[ph::dec][:L]
    y = np.roll(yy, -lag)
    g = np.vdot(ref, y) / np.vdot(ref, ref)
    evm = np.linalg.norm(y - g * ref) / np.linalg.norm(g * ref)
    sps = max(1, 16 // dec)
    yr = y / g
    agree = 0.0
    for o in range(sps):
        idx = np.arange(o, L, sps)
        live = np.abs(ref[idx]) > 0.3 * np.sqrt(np.mean(np.abs(ref) ** 2))
        ok = np.mean(((np.sign(yr[idx].real) == np.sign(ref[idx].real)) &
                      (np.sign(yr[idx].imag) == np.sign(ref[idx].imag))) |
                     ~live)
        agree = max(agree, float(ok))
    return rho, evm, agree, cfo, loops


def analyse_packet(bursts, fs, centre, ref16, want_cfo):
    """Each burst matched to the PC-built loop; the median over bursts."""
    r80 = ref16.astype(np.float64)
    r80 = r80[0::2] + 1j * r80[1::2]
    dec = 80e6 / fs
    if abs(dec - round(dec)) > 1e-6:
        return {"note": "packet check needs a rate that divides 80 MS/s"}
    dec = int(round(dec))
    rs = [r for r in (packet_one(x, fs, centre, r80, dec, want_cfo)
                      for x in bursts) if r]
    if not rs:
        return {"note": "bursts too short for the packet check"}
    a = np.array([r[:4] for r in rs])
    med = np.median(a, axis=0)
    return {"carrier_offset_hz": round(med[3]),
            "correlation": round(float(med[0]), 4),
            "evm_percent": round(100 * float(med[1]), 1),
            "qpsk_agree_percent": round(100 * float(med[2]), 2),
            "best_correlation": round(float(a[:, 0].max()), 4),
            "bursts": len(rs), "loops_per_burst": rs[0][4]}


# ------------------------------------------------------------- plotting

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
S1, S2 = "#2a78d6", "#eb6834"

# Per test: title, target bin width (Hz), frequency span (MHz), dB span.
PLOTS = {
    "tone": ("Tone example: 1 MHz tone above a 2442 MHz carrier",
             20e3, (2431, 2451), (-70, 5)),
    "fmtone": ("FmTone example: 1 kHz audio, 75 kHz deviation, 250 kHz above "
               "the carrier", 2.4e3, (2441.75, 2442.75), (-60, 5)),
    "packet": ("Packet example: QPSK, 5 Msym/s, RRC 0.35, at 2442 MHz",
               40e3, (2430, 2450), (-60, 8)),
}


def marks_for(name, off, centre):
    o = off / 1e6
    c = centre / 1e6
    m = [(c, "receiver\nDC", "right", -40 if name == "tone" else -18)]
    if name == "tone":
        m += [(TONE / 1e6 + o, "tone\n2443 MHz", "left", -8),
              (CARRIER / 1e6 + o, "leak", "left", -22),
              (TONE_IMAGE / 1e6 + o, "image\n2441 MHz", "right", -22),
              (2 * c - TONE / 1e6 - o, "receiver's\nimage of\nthe tone",
               "right", -10)]
    elif name == "fmtone":
        m = [(CARRIER / 1e6 + o, "carrier leak 2442 MHz", "right", -10),
             ((FM - FM_DEV) / 1e6 + o, "-75 kHz", "right", -48),
             ((FM + FM_DEV) / 1e6 + o, "+75 kHz", "left", -48),
             (FM / 1e6 + o, "2442.25 MHz", "center", 3.2)]
    else:
        m += [((CARRIER - PKT_HALF) / 1e6, "-3.375 MHz", "left", -52),
              ((CARRIER + PKT_HALF) / 1e6, "+3.375 MHz", "right", -52),
              (2 * c - CARRIER / 1e6, "receiver's\nimage", "center", -12)]
    return m


def plot(path, bursts, fs, centre, name, sub, off, ref16):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
        "xtick.color": INK2, "ytick.color": INK2, "text.color": INK,
        "axes.spines.top": False, "axes.spines.right": False})
    title, bin_hz, xlim, ylim = PLOTS[name]
    f, p = welch(bursts, fs, nfft_for(fs, bin_hz, len(bursts[0])))
    fm = (centre + f) / 1e6
    xlim = (max(xlim[0], fm[0]), min(xlim[1], fm[-1]))
    d = 10 * np.log10(p + 1e-30)
    keep = (fm >= xlim[0]) & (fm <= xlim[1])
    d = d - d[keep].max()
    fig, ax = plt.subplots(figsize=(9, 4.1), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.plot(fm, d, color=S1, linewidth=1.2, label="recording")
    if name == "packet" and ref16 is not None:
        r = ref16.astype(np.float64)
        r = np.tile(r[0::2] + 1j * r[1::2], 8)
        rf, rp = welch([r], 80e6, 1024)
        rf = (CARRIER + rf) / 1e6
        rdb = 10 * np.log10(rp + 1e-30)
        k = np.abs(rf - CARRIER / 1e6) <= PKT_HALF / 1e6 + 0.125
        band = np.abs(fm - CARRIER / 1e6) <= 2
        rband = np.abs(rf - CARRIER / 1e6) <= 2
        o = np.median(d[band]) - np.median(rdb[rband])
        ax.plot(rf[k], rdb[k] + o, color=S2, linewidth=1.2,
                linestyle=(0, (5, 3)), label="PC reference (espsdrtx_sc_packet)")
        leg = ax.legend(loc="upper left", frameon=False, fontsize=9)
        for t in leg.get_texts():
            t.set_color(INK2)
    for mx, text, ha, y in marks_for(name, off, centre):
        if not xlim[0] < mx < xlim[1]:
            continue
        ax.axvline(mx, color=INK2, linewidth=0.8, linestyle=":", zorder=1)
        dx = {"left": 4, "right": -4, "center": 0}[ha]
        ax.annotate(text, xy=(mx, y), xytext=(dx, 0), textcoords="offset points",
                    ha=ha, va="center", fontsize=8, color=INK2,
                    linespacing=1.1)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_xlabel("Frequency (MHz)")
    ax.set_ylabel("dB relative to the strongest bin")
    ax.ticklabel_format(axis="x", useOffset=False)
    fig.text(0.01, 0.978, title, fontsize=12, fontweight="bold", color=INK,
             va="top")
    fig.text(0.01, 0.918, sub, fontsize=8.5, color=INK2, va="top",
             linespacing=1.3)
    fig.subplots_adjust(left=0.08, right=0.965, top=0.75, bottom=0.12)
    fig.savefig(path, facecolor=SURFACE)
    plt.close(fig)


# ---------------------------------------------------------------- tests

def describe(name, args):
    who = args.board + (f" ({args.chip})" if args.chip else "")
    return {
        "tone": f"espsdrtx Tone example on {who}: carrier 2442 MHz, complex "
                "tone 1 MHz above it (2443 MHz), amplitude 100, 80 MS/s loop "
                "of 16000 words.",
        "fmtone": f"espsdrtx FmTone example on {who}: CPU-paced FM stream, "
                  "2 MS/s, 1 kHz audio tone, 75 kHz deviation, 250 kHz above "
                  "a 2442 MHz carrier, amplitude 50.",
        "packet": f"espsdrtx Packet example on {who}: QPSK, 5 Msym/s (16 "
                  "samples per symbol at 80 MS/s), RRC 0.35, rms word 25, "
                  "payload 'hello from espsdrtx'; one loop repeated, carrier "
                  "2442 MHz. packet_qpsk_reference.cs16 holds the same loop "
                  "built on a PC.",
    }[name]


def analyse(name, bursts, fs, centre, state, ref16):
    if name == "tone":
        res = analyse_tone(bursts, fs, centre)
        state["offset"] = res["carrier_offset_hz"]
    elif name == "fmtone":
        res = analyse_fm(bursts, fs, centre, state.get("offset"))
    else:
        res = analyse_packet(bursts, fs, centre, ref16, state.get("offset"))
    return res


def save(args, name, raws, dtype, full, fs, centre, hw, rx_desc, results,
         state, ref16, stamps):
    t = TESTS[name]
    base = os.path.join(args.out, f"{t['base']}_{args.suffix}")
    np.concatenate(raws).tofile(base + ".sigmf-data")
    g = {"core:datatype": dtype, "core:sample_rate": fs,
         "core:version": "1.0.0",
         "core:description": describe(name, args),
         "core:hw": hw, "core:recorder": "src/hardware/record.py",
         "core:license": "https://www.gnu.org/licenses/gpl-3.0.html",
         "espsdrtx:example": "examples/" + t["example"],
         "espsdrtx:board": args.board, "espsdrtx:esp_carrier_hz": CARRIER,
         "espsdrtx:full_scale": full, "espsdrtx:results": results}
    for k, v in (("espsdrtx:chip", args.chip),
                 ("espsdrtx:esptool", args.chip_info),
                 ("espsdrtx:note", args.note)):
        if v:
            g[k] = v
    if len(raws) > 1:
        g["espsdrtx:bursts"] = (f"{len(raws)} bursts of {len(raws[0]) // 2} "
                                "samples, one capture segment each, not "
                                "contiguous in time")
    caps, start = [], 0
    for raw, stamp in zip(raws, stamps):
        caps.append({"core:sample_start": start, "core:frequency": centre,
                     "core:datetime": stamp})
        start += len(raw) // 2
    with open(base + ".sigmf-meta", "w") as fh:
        json.dump({"global": g, "captures": caps, "annotations": []}, fh,
                  indent=2)
        fh.write("\n")
    if name == "packet" and ref16 is not None:
        ref16.tofile(os.path.join(args.out, "packet_qpsk_reference.cs16"))

    bursts = [iq(r, full) for r in raws]
    nfft = nfft_for(fs, PLOTS[name][1], len(bursts[0]))
    off = state.get("offset", results.get("carrier_offset_hz", 0)) or 0
    line3 = (f"{len(bursts)} x {len(bursts[0]) / fs * 1e3:.2f} ms, "
             f"{nfft}-point Welch ({fs / nfft / 1e3:.1f} kHz bins)")
    if off and name != "packet":
        line3 += f"; markers {off / 1e3:+.1f} kHz (crystal offset)"
    plot(base + ".png", bursts, fs, centre, name,
         "\n".join((args.board, rx_desc, line3)),
         off, ref16)
    log(f"  wrote {os.path.relpath(base, REPO)}.sigmf-data/.sigmf-meta/.png")


FM_FS = 2e6                  # FmTone recordings are saved at this rate


def shrink(raws, full, fs, centre, new_centre, factor):
    """Shift each burst to new_centre and decimate it by factor.

    The low-pass (161-tap Kaiser, beta 8) is flat to +-0.75 MHz at 2 MS/s
    out and about 80 dB down from where aliases would land in the band kept.
    Returns int16 bursts with a full scale 16 times the receiver's, so the
    rounding adds no noise.
    """
    ntap = 161
    cut = 0.375 * fs / factor
    k = np.arange(ntap) - (ntap - 1) / 2
    h = np.sinc(2 * cut / fs * k) * np.kaiser(ntap, 8)
    h /= h.sum()
    out_full = 16 * full
    out = []
    for raw in raws:
        x = iq(raw, full)
        x = x * np.exp(-2j * np.pi * (new_centre - centre) *
                       np.arange(len(x)) / fs)
        y = np.convolve(x, h, "valid")[::factor] * out_full
        r = np.empty(2 * len(y), dtype=np.int16)
        r[0::2], r[1::2] = np.round(y.real), np.round(y.imag)
        out.append(r)
    return out, out_full


def now():
    return datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")


def record_test(args, rx, name, state, ref16):
    t = TESTS[name]
    log(f"\n== {name}: examples/{t['example']}")
    if args.fqbn and args.port:
        flash(args, t["example"])
    elif not args.yes:
        input(f"  Flash examples/{t['example']} to the board under test, "
              "then press Enter ... ")
    rate = next((r for r in t["rates"]
                 if any(abs(r - x) < 1e3 for x in rx.rates)), rx.rates[0])
    rx.setup(t["rx_mhz"], rate)
    fs = RATE_CODES[rx.code]
    n = rx.max_n
    log(f"  receiver at {t['rx_mhz']} MHz, {fs / 1e6:g} MS/s, bursts of {n}")
    raws, full, gain = record_bursts(rx, t, n)
    stamps = [now()] * len(raws)
    centre = t["rx_mhz"] * 1e6
    shrunk = ""
    rx_desc = (f"{rx.family} (ESP-SDR) at {t['rx_mhz']} MHz, {fs / 1e6:g} "
               f"MS/s, gain index {gain}")
    if name == "fmtone" and abs(fs / FM_FS - round(fs / FM_FS)) < 1e-6 \
            and fs > FM_FS:
        raws, full = shrink(raws, full, fs, centre, FM, int(round(fs / FM_FS)))
        shrunk = (f"; then on the PC shifted from {centre / 1e6:g} to "
                  f"{FM / 1e6:g} MHz and decimated {fs / FM_FS:g}x to "
                  f"{FM_FS / 1e6:g} MS/s (161-tap Kaiser low-pass, flat to "
                  "+-0.75 MHz) to keep the file small")
        fs, centre = FM_FS, FM
        rx_desc += f", cut to {FM_FS / 1e6:g} MS/s around {FM / 1e6:g} MHz"
    res = analyse(name, [iq(r, full) for r in raws], fs, centre, state, ref16)
    res["receiver_gain_index"] = gain
    v = rx.version or {}
    ver = f"{v.get('revision', '?')[:12]} ({v.get('build_date', '?')})"
    hw = (f"{rx.family} running ESPARGOS ESP-SDR {ver}" +
          f", manual gain index {gain} (of {rx.gain_lim[0]}..."
          f"{rx.gain_lim[1]}), widest bandwidth, {rx.bits}-bit samples; "
          "over the air" + shrunk)
    save(args, name, raws, "ci16_le", full, fs, centre, hw, rx_desc, res,
         state, ref16, stamps)
    return res


def analyse_input(args, name, state, ref16):
    """--input: a SigMF recording made earlier, by any receiver."""
    path = args.input
    meta_path = re.sub(r"\.sigmf-data$", ".sigmf-meta", path)
    if meta_path == path or not os.path.exists(meta_path):
        die("--input takes a .sigmf-data file with its .sigmf-meta beside it")
    meta = json.load(open(meta_path))
    dtype = meta["global"]["core:datatype"]
    fs = meta["global"]["core:sample_rate"]
    caps = meta["captures"]
    centre = caps[0]["core:frequency"]
    np_t = {"ci8": np.int8, "ci16_le": np.int16, "cf32_le": np.float32}
    if dtype not in np_t:
        die("datatype must be ci8, ci16_le or cf32_le")
    full = meta["global"].get("espsdrtx:full_scale",
                              {"ci8": 128, "ci16_le": 32768, "cf32_le": 1}[dtype])
    raw = np.fromfile(path, dtype=np_t[dtype])
    starts = [c["core:sample_start"] for c in caps] + [len(raw) // 2]
    raws = [raw[2 * a:2 * b] for a, b in zip(starts, starts[1:])]
    t = TESTS[name]
    if len(raws) == 1:            # one long recording: the cleanest stretch
        x = iq(raws[0], full)
        start, length, _ = burst_check(x, fs, centre, t["band"], 0.1)
        if length < 4:
            die(f"the {name} signal was not found in {path}")
        raws = [raws[0][2 * start * SEG:2 * (start + length) * SEG]]
    res = analyse(name, [iq(r, full) for r in raws], fs, centre, state, ref16)
    log("  " + json.dumps(res))
    if args.out:
        save(args, name, raws, dtype, full, fs, centre,
             meta["global"].get("core:hw", ""),
             f"Receiver at {centre / 1e6:g} MHz, {fs / 1e6:g} MS/s", res,
             state, ref16,
             [c.get("core:datetime", "") for c in caps])
    return res


# -------------------------------------------------------------- summary

def summary(args, results, rx_line):
    lines = [f"## Recordings: {args.board}", ""]
    if args.chip_info:
        lines += ["Board under test (esptool): " + ", ".join(
            f"{k} {v}" for k, v in args.chip_info.items()), ""]
    if rx_line:
        lines += ["Receiver: " + rx_line, ""]
    lines += ["| Test | Result |", "|---|---|"]
    for name, r in results.items():
        if "note" in r:
            s = r["note"]
        elif name == "tone":
            s = (f"tone {r['tone_over_floor_db']} dB over the floor; carrier "
                 f"leak {r.get('leak_dbc')} dBc, image {r.get('image_dbc')} "
                 f"dBc; crystal offset {r['carrier_offset_hz'] / 1e3:+.1f} "
                 "kHz against the receiver")
        elif name == "fmtone":
            s = (f"carrier leak {r['leak_dbc_of_fm']} dBc of the FM power; "
                 f"99 % bandwidth {r['occupied_bw_99_khz']} kHz")
        else:
            s = (f"correlation {r['correlation']} with the PC-built packet "
                 f"(best {r['best_correlation']}), EVM {r['evm_percent']} %, "
                 f"QPSK decisions {r['qpsk_agree_percent']} % the same; "
                 f"median of {r['bursts']} bursts")
        lines.append(f"| {name} | {s} |")
    lines += ["", "For comparison (recorded with a HackRF): the ESP32-C3 and "
              "ESP32-C6 boards gave a Tone leak of -31 and -37 dBc and image "
              "of -34 and -52 dBc; an FmTone leak of -23 dBc on both; a Packet "
              "correlation of 0.986 and 0.995, EVM 17 % and 10 %. An ESP "
              "receiver adds its own noise and image, so expect somewhat "
              "worse figures from it.", ""]
    text = "\n".join(lines)
    path = os.path.join(args.out, f"results_{args.suffix}.md")
    open(path, "w").write(text)
    with open(os.path.join(args.out, f"results_{args.suffix}.json"),
              "w") as fh:
        json.dump({"board": args.board, "chip": args.chip,
                   "esptool": args.chip_info, "receiver": rx_line,
                   "results": results}, fh, indent=2)
        fh.write("\n")
    log("\n" + text)
    log(f"Wrote {os.path.relpath(path, REPO)} (and .json). Paste it into your "
        "pull request along with the files in "
        f"{os.path.relpath(args.out, REPO)}/.")


def slug(s):
    return re.sub(r"[^a-z0-9]+", "_", s.lower()).strip("_")


def main():
    ap = argparse.ArgumentParser(
        description="Record espsdrtx's examples from an ESP32 board, using a "
        "second ESP32 running ESPARGOS ESP-SDR as the receiver.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("--board", required=True,
                    help='the board under test, e.g. "Waveshare ESP32-S3-Zero"')
    ap.add_argument("--chip", default="",
                    help="its ESP-IDF target, e.g. esp32s3; picks the output "
                    "folder src/hardware/<chip>/captures")
    ap.add_argument("--rx-port", help="the ESP-SDR receiver's serial port")
    ap.add_argument("--rx-baud", type=int, default=2000000,
                    help="for a UART receiver (default 2000000)")
    ap.add_argument("--tests", nargs="+", default=list(TESTS),
                    choices=list(TESTS))
    ap.add_argument("--fqbn", help="Arduino board of the board under test, "
                    "e.g. esp32:esp32:esp32s3 (with --port: flash each "
                    "example)")
    ap.add_argument("--port", help="the board under test's serial port")
    ap.add_argument("--arduino-cli", default="arduino-cli")
    ap.add_argument("--yes", action="store_true",
                    help="do not ask to flash: the example is already running "
                    "(one test)")
    ap.add_argument("--out", help="output folder (default "
                    "src/hardware/<chip>/captures)")
    ap.add_argument("--suffix", help="file name suffix (default: from --board)")
    ap.add_argument("--note", help="anything else for the metadata: antennas, "
                    "distance, cable and attenuator ...")
    ap.add_argument("--input", help="analyse this .sigmf-data instead of "
                    "recording (one test)")
    args = ap.parse_args()

    if args.input and len(args.tests) != 1:
        die("with --input, name the one test it holds: --tests tone")
    if not args.input and not args.rx_port:
        die("give the receiver's port with --rx-port")
    if args.yes and len(args.tests) != 1 and not (args.fqbn and args.port):
        die("--yes runs one test: name it with --tests")
    if bool(args.fqbn) != bool(args.port):
        die("--fqbn and --port go together")
    if args.port and args.rx_port and args.port == args.rx_port:
        die("--port is the board under test, --rx-port the receiver: they "
            "must differ")
    args.suffix = args.suffix or slug(args.board)
    if not args.out and not args.input:
        if not args.chip:
            die("give --chip (or --out)")
        args.out = os.path.join(HERE, args.chip, "captures")
    if args.out:
        os.makedirs(args.out, exist_ok=True)
    if args.fqbn and not shutil.which(args.arduino_cli) and \
            not os.path.exists(args.arduino_cli):
        die("arduino-cli not found (give its path with --arduino-cli)")

    ref16 = packet_reference() if "packet" in args.tests else None
    state, results = {}, {}
    order = [n for n in TESTS if n in args.tests]   # tone first: the offset
    if args.input:
        args.chip_info = {}
        for name in order:
            results[name] = analyse_input(args, name, state, ref16)
        return

    args.chip_info = chip_info(args.port) if args.port else {}
    if args.chip_info:
        log("board under test: " + ", ".join(f"{k} {v}" for k, v in
                                              args.chip_info.items()))
    rx = EspSdr(args.rx_port, args.rx_baud)
    rx_line = (f"{rx.family}, ESP-SDR {json.dumps(rx.version)}, gain indices "
               f"{rx.gain_lim[0]} to {rx.gain_lim[1]}")
    log("receiver: " + rx_line)
    try:
        for name in order:
            results[name] = record_test(args, rx, name, state, ref16)
            log("  " + json.dumps(results[name]))
    finally:
        rx.release()
    summary(args, results, rx_line)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit("\nstopped")

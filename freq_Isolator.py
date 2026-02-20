#!/usr/bin/env python3
import argparse
import time
import math
import busio
import board
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np


@dataclass
class Peak:
    freq_hz: float
    amp: float
    rel_amp: float

i2c = busio.I2C(board.SCL, board.SDA)
imu = LSM6DSO32(i2c)

def read_accel_sample() -> float:
    """
    Return ONE scalar acceleration sample (e.g., magnitude, or a single axis) in m/s^2.
    """
    ax, ay, az = imu.acceleration

    return ax


def read_accel_sample_simulated(t: float) -> float:
    """
    Simulated accel: 2 sinusoids + noise.
    Useful for testing FFT/peak picking without hardware.
    """
    a = 0.8 * math.sin(2 * math.pi * 37.0 * t)      # 37 Hz
    b = 0.3 * math.sin(2 * math.pi * 121.0 * t)     # 121 Hz
    n = 0.05 * np.random.randn()
    return a + b + n


def collect_time_series(duration_s: float, fs_hz: float, simulate: bool) -> Tuple[np.ndarray, float]:
    """
    Collect N = duration*fs samples at ~fs rate.
    Returns samples array and measured sampling rate (based on timestamps).
    """
    if duration_s <= 0:
        raise ValueError("duration must be > 0")
    if fs_hz <= 0:
        raise ValueError("fs must be > 0")

    n_target = int(round(duration_s * fs_hz))
    if n_target < 8:
        raise ValueError("duration*fs too small; need at least ~8 samples")

    samples = np.zeros(n_target, dtype=float)
    t_stamps = np.zeros(n_target, dtype=float)

    period = 1.0 / fs_hz
    t0 = time.perf_counter()
    next_t = t0


    for i in range(n_target):
        now = time.perf_counter()

        if now < next_t:
            time.sleep(next_t - now)
            now = time.perf_counter()

        t = now - t0

        t_before_read = time.perf_counter()
        if simulate:
            val = read_accel_sample_simulated(t)
        else:
            val = read_accel_sample()
        t_after_read = time.perf_counter()

        samples[i] = val

        if i % 100 == 0:
            read_dt_ms = (t_after_read - t_before_read) * 1000
            print(f"i={i}  t={t:.3f}s  read_dt={read_dt_ms:.2f} ms")

        t_stamps[i] = now
        next_t += period

    # estimate actual fs from timestamps
    dt = np.diff(t_stamps)
    fs_meas = 1.0 / np.mean(dt) if len(dt) > 0 else fs_hz
    return samples, fs_meas


def single_sided_fft_amplitude(x: np.ndarray, fs_hz: float, window: str = "hann") -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns (freqs, amps) for single-sided amplitude spectrum.
    Amplitude is scaled so a pure sine of amplitude A ideally shows ~A at its bin (with caveats).
    """
    n = len(x)

    # remove mean (kills DC bias)
    x = x - np.mean(x)

    # windowing
    if window == "hann":
        w = np.hanning(n)
    elif window == "rect":
        w = np.ones(n)
    else:
        raise ValueError("window must be 'hann' or 'rect'")

    xw = x * w

    # FFT (real)
    X = np.fft.rfft(xw)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs_hz)

    # Amplitude scaling:
    # - rfft gives N/2+1 bins
    # - For single-sided spectrum, multiply non-DC, non-Nyquist bins by 2
    # - Also correct for window coherent gain (mean of window)
    cg = np.mean(w)  # coherent gain
    amps = (np.abs(X) / (n * cg))

    if n > 1:
        amps[1:-1] *= 2.0  # double interior bins
    return freqs, amps


def pick_top_peaks(
    freqs: np.ndarray,
    amps: np.ndarray,
    k: int = 3,
    min_freq_hz: float = 1.0,
    max_freq_hz: Optional[float] = None,
    guard_bins: int = 2
) -> List[Peak]:
    """
    Simple peak picking:
    - ignores freqs < min_freq_hz
    - optionally ignores freqs > max_freq_hz
    - finds local maxima and then takes top-k by amplitude
    - uses a guard band (suppresses neighbors around chosen peaks)
    """
    if len(freqs) != len(amps):
        raise ValueError("freqs and amps must match length")

    # mask band
    mask = freqs >= min_freq_hz
    if max_freq_hz is not None:
        mask &= (freqs <= max_freq_hz)

    idxs = np.where(mask)[0]
    if len(idxs) < 3:
        return []

    # local maxima: amp[i-1] < amp[i] >= amp[i+1]
    candidates = []
    for i in idxs[1:-1]:
        if amps[i] > amps[i - 1] and amps[i] >= amps[i + 1]:
            candidates.append(i)

    if not candidates:
        return []

    # sort candidates by amplitude descending
    candidates.sort(key=lambda i: amps[i], reverse=True)

    chosen = []
    suppressed = np.zeros(len(amps), dtype=bool)

    for i in candidates:
        if suppressed[i]:
            continue
        chosen.append(i)
        lo = max(0, i - guard_bins)
        hi = min(len(amps), i + guard_bins + 1)
        suppressed[lo:hi] = True
        if len(chosen) >= k:
            break

    peak_amps = [amps[i] for i in chosen]
    max_amp = max(peak_amps) if peak_amps else 1.0

    peaks = [
        Peak(freq_hz=float(freqs[i]), amp=float(amps[i]), rel_amp=float(amps[i] / max_amp))
        for i in chosen
    ]
    return peaks


def main():
    ap = argparse.ArgumentParser(description="Collect accel data, FFT, report top dominant frequencies.")
    ap.add_argument("--duration", type=float, default=30.0, help="seconds to record (default 30)")
    ap.add_argument("--fs", type=float, default=800.0, help="target sampling rate in Hz (default 800)")
    ap.add_argument("--axis", type=str, default="scalar", help="placeholder; use if you later add x/y/z")
    ap.add_argument("--min-freq", type=float, default=1.0, help="ignore peaks below this frequency (Hz)")
    ap.add_argument("--max-freq", type=float, default=None, help="ignore peaks above this frequency (Hz)")
    ap.add_argument("--window", type=str, default="hann", choices=["hann", "rect"], help="FFT window")
    ap.add_argument("--simulate", action="store_true", help="use simulated accel instead of hardware")
    ap.add_argument("--save-spectrum", type=str, default=None, help="path to save spectrum CSV (freq, amp)")
    args = ap.parse_args()

    x, fs_meas = collect_time_series(args.duration, args.fs, args.simulate)

    freqs, amps = single_sided_fft_amplitude(x, fs_meas, window=args.window)

    peaks = pick_top_peaks(
        freqs=freqs,
        amps=amps,
        k=3,
        min_freq_hz=args.min_freq,
        max_freq_hz=args.max_freq,
        guard_bins=2
    )

    print(f"Requested duration: {args.duration:.3f} s")
    print(f"Target fs: {args.fs:.2f} Hz | Measured fs: {fs_meas:.2f} Hz")
    print(f"FFT bins: {len(freqs)} | Nyquist: {fs_meas/2:.2f} Hz")
    print()

    if not peaks:
        print("No peaks found in the specified band.")
    else:
        print("Top dominant frequencies:")
        for j, p in enumerate(peaks, start=1):
            print(f"{j}) {p.freq_hz:8.3f} Hz | amp={p.amp:.6g} | rel={p.rel_amp:.3f}")

    if args.save_spectrum:
        data = np.column_stack([freqs, amps])
        np.savetxt(args.save_spectrum, data, delimiter=",", header="freq_hz,amp", comments="")
        print()
        print(f"Saved spectrum CSV to: {args.save_spectrum}")


if __name__ == "__main__":
    main()

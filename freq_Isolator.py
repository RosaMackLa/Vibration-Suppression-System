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

# ── matplotlib setup ──────────────────────────────────────────────────────────
# Must set backend BEFORE importing pyplot, so the import is deferred to
# plot_results() and the backend is set right beforehand.  This keeps the
# rest of the module importable on a headless Pi even without a display.
# ─────────────────────────────────────────────────────────────────────────────


@dataclass
class Peak:
    freq_hz: float
    amp: float
    rel_amp: float

_imu = None

def get_imu():
    global _imu
    if _imu is None:
        i2c = busio.I2C(board.SCL, board.SDA)
        _imu = LSM6DSO32(i2c)
    return _imu

def read_accel_sample() -> float:
    """
    Returns scalar acceleration sample from x-axis in m/s^2.
    """
    imu = get_imu()
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


def collect_time_series(duration_s: float, fs_hz: float, simulate: bool) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    Collect N = duration*fs samples at ~fs rate.
    Returns (samples, timestamps, measured_fs).
    Timestamps are seconds elapsed from t0.
    """
    if duration_s <= 0:
        raise ValueError("duration must be > 0")
    if fs_hz <= 0:
        raise ValueError("fs must be > 0")

    n_target = int(round(duration_s * fs_hz))
    if n_target < 8:
        raise ValueError("duration*fs too small; need at least ~8 samples")

    samples  = np.zeros(n_target, dtype=float)
    t_stamps = np.zeros(n_target, dtype=float)

    period = 1.0 / fs_hz
    t0     = time.perf_counter()
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

        samples[i]  = val
        t_stamps[i] = t          # elapsed seconds from t0

        if i % 100 == 0:
            read_dt_ms = (t_after_read - t_before_read) * 1000
            print(f"i={i}  t={t:.3f}s  read_dt={read_dt_ms:.2f} ms")

        next_t += period

    # estimate actual fs from timestamps
    dt     = np.diff(t_stamps)
    fs_meas = 1.0 / np.mean(dt) if len(dt) > 0 else fs_hz
    return samples, t_stamps, fs_meas


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
    X    = np.fft.rfft(xw)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs_hz)

    # Amplitude scaling:
    # - rfft gives N/2+1 bins
    # - For single-sided spectrum, multiply non-DC, non-Nyquist bins by 2
    # - Also correct for window coherent gain (mean of window)
    cg   = np.mean(w)  # coherent gain
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

    chosen     = []
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
    max_amp   = max(peak_amps) if peak_amps else 1.0

    peaks = [
        Peak(freq_hz=float(freqs[i]), amp=float(amps[i]), rel_amp=float(amps[i] / max_amp))
        for i in chosen
    ]
    return peaks


def plot_results(
    samples: np.ndarray,
    t_stamps: np.ndarray,
    fs_meas: float,
    freqs: np.ndarray,
    amps: np.ndarray,
    peaks: List[Peak],
    out_path: str,
    min_freq_hz: float,
    max_freq_hz: Optional[float],
) -> None:
    """
    Save a two-panel figure:
      Top:    raw acceleration time series
      Bottom: single-sided FFT amplitude spectrum with peak markers

    Uses the Agg (file-only) backend so this works on a headless Pi.
    """
    import matplotlib
    matplotlib.use("Agg")           # must be set before importing pyplot
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(10, 7))
    fig.suptitle("VSS — Accelerometer Capture & FFT", fontweight="bold")

    # ── Panel 1: time series ──────────────────────────────────────────────────
    ax_t = axes[0]
    ax_t.plot(t_stamps, samples, linewidth=0.6, color="steelblue")
    ax_t.set_xlabel("Time (s)")
    ax_t.set_ylabel("Acceleration (m/s²)")
    ax_t.set_title("Raw Acceleration Time Series")
    ax_t.grid(True, linestyle="--", alpha=0.5)

    # ── Panel 2: FFT spectrum ─────────────────────────────────────────────────
    ax_f = axes[1]

    freq_lo   = min_freq_hz
    freq_hi   = max_freq_hz if max_freq_hz is not None else freqs[-1]
    band_mask = (freqs >= freq_lo) & (freqs <= freq_hi)

    ax_f.plot(freqs[band_mask], amps[band_mask], linewidth=0.8, color="darkorange")
    ax_f.set_xlabel("Frequency (Hz)")
    ax_f.set_ylabel("Amplitude (m/s²)")
    ax_f.set_title("Single-Sided FFT Amplitude Spectrum")
    ax_f.grid(True, linestyle="--", alpha=0.5)

    for p in peaks:
        ax_f.axvline(p.freq_hz, color="crimson", linewidth=0.8, linestyle="--", alpha=0.7)
        ax_f.annotate(
            f"{p.freq_hz:.1f} Hz\n{p.amp:.3g}",
            xy=(p.freq_hz, p.amp),
            xytext=(6, 6),
            textcoords="offset points",
            fontsize=7,
            color="crimson",
        )
        ax_f.scatter([p.freq_hz], [p.amp], color="crimson", s=30, zorder=5)

    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved to: {out_path}")


def save_data_csv(t_stamps: np.ndarray, samples: np.ndarray, path: str) -> None:
    """
    Save raw time-series data as a two-column CSV:
        time_s, accel_ms2

    This is the primary export for post-processing in MATLAB or other tools.
    The timestamp column contains elapsed seconds from t0 (not wall-clock time),
    matching the values used for the Python plot.
    """
    data = np.column_stack([t_stamps, samples])
    np.savetxt(
        path,
        data,
        delimiter=",",
        header="time_s,accel_ms2",
        comments="",   # suppress the leading '#' numpy adds by default
        fmt="%.9f",    # enough precision to recover fs accurately on reload
    )
    print(f"Raw data saved to:  {path}  ({len(samples)} samples)")


def main():
    ap = argparse.ArgumentParser(description="Collect accel data, FFT, report top dominant frequencies.")
    ap.add_argument("--duration",      type=float, default=30.0,  help="seconds to record (default 30)")
    ap.add_argument("--fs",            type=float, default=800.0, help="target sampling rate in Hz (default 800)")
    ap.add_argument("--axis",          type=str,   default="scalar", help="placeholder; use if you later add x/y/z")
    ap.add_argument("--min-freq",      type=float, default=1.0,   help="ignore peaks below this frequency (Hz)")
    ap.add_argument("--max-freq",      type=float, default=None,  help="ignore peaks above this frequency (Hz)")
    ap.add_argument("--window",        type=str,   default="hann", choices=["hann", "rect"], help="FFT window")
    ap.add_argument("--simulate",      action="store_true",        help="use simulated accel instead of hardware")
    ap.add_argument("--save-data",     type=str,   default=None,  help="path to save raw time-series CSV (time_s, accel_ms2)")
    ap.add_argument("--save-spectrum", type=str,   default=None,  help="path to save spectrum CSV (freq_hz, amp)")
    ap.add_argument("--plot",          action="store_true",        help="save a two-panel plot (time series + FFT spectrum)")
    ap.add_argument("--plot-out",      type=str,   default="spectrum_plot.png",
                    help="output path for the plot image (default: spectrum_plot.png)")
    args = ap.parse_args()

    # ── Collect ──────────────────────────────────────────────────────────────
    x, t_stamps, fs_meas = collect_time_series(args.duration, args.fs, args.simulate)

    # ── Spectral analysis ─────────────────────────────────────────────────────
    freqs, amps = single_sided_fft_amplitude(x, fs_meas, window=args.window)

    peaks = pick_top_peaks(
        freqs=freqs,
        amps=amps,
        k=3,
        min_freq_hz=args.min_freq,
        max_freq_hz=args.max_freq,
        guard_bins=2
    )

    # ── Console report ────────────────────────────────────────────────────────
    print(f"\nRequested duration : {args.duration:.3f} s")
    print(f"Target fs          : {args.fs:.2f} Hz | Measured fs: {fs_meas:.2f} Hz")
    print(f"FFT bins           : {len(freqs)} | Nyquist: {fs_meas/2:.2f} Hz")
    print()

    if not peaks:
        print("No peaks found in the specified band.")
    else:
        print("Top dominant frequencies:")
        for j, p in enumerate(peaks, start=1):
            print(f"  {j}) {p.freq_hz:8.3f} Hz | amp={p.amp:.6g} | rel={p.rel_amp:.3f}")

    # ── Exports ───────────────────────────────────────────────────────────────
    if args.save_data:
        save_data_csv(t_stamps, x, args.save_data)

    if args.save_spectrum:
        data = np.column_stack([freqs, amps])
        np.savetxt(args.save_spectrum, data, delimiter=",",
                   header="freq_hz,amp", comments="", fmt="%.9f")
        print(f"Spectrum CSV saved: {args.save_spectrum}")

    if args.plot:
        plot_results(
            samples=x,
            t_stamps=t_stamps,
            fs_meas=fs_meas,
            freqs=freqs,
            amps=amps,
            peaks=peaks,
            out_path=args.plot_out,
            min_freq_hz=args.min_freq,
            max_freq_hz=args.max_freq,
        )


if __name__ == "__main__":
    main()

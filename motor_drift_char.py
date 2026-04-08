#!/usr/bin/env python3
"""
motor_drift_char.py  —  DC motor frequency stability characterization
═════════════════════════════════════════════════════════════════════

PURPOSE
───────
Answers one question: how stable is the DC disturbance motor's frequency
over minutes?  The VSS controller has never had this measured — every
plant ID and cancellation run has assumed a single-shot FFT is good
enough.  This script tests that assumption directly.

Logs the IMU x-axis at FS Hz for TOTAL_SEC seconds, splits the log into
WINDOW_SEC-long chunks, FFTs each chunk with parabolic interpolation for
sub-bin resolution, extracts the dominant peak in [F_MIN, F_MAX], and
plots the peak frequency vs time plus a recommendation.

USAGE
─────
  # 1. Start the DC motor at target speed (8-9 Hz for VSS operating window)
  # 2. Wait ~2 min for thermal stabilization
  # 3. Run:
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 motor_drift_char.py

  # Optional flags:
  #   --duration 300    total log length in seconds (default 300 = 5 min)
  #   --window   10     FFT window length in seconds (default 10)
  #   --fmin     5      search band lower bound Hz (default 5)
  #   --fmax     15     search band upper bound Hz (default 15)

OUTPUTS
───────
  motor_drift_out/
    drift_log.csv    — per-window (t_center, f_peak, amp_peak)
    drift_plot.png   — 2-panel plot: frequency vs time, amplitude vs time
    summary.txt      — stats + actionable recommendation

INTERPRETATION (automated in summary.txt, but for reference)
────────────────────────────────────────────────────────────
  σ < 0.02 Hz, no trend      →  rock-solid; --manual_freqs fine
  σ ~ 0.02-0.05 Hz           →  mild drift; gradient tracker ok with µ_omega≥1e-5
  σ ~ 0.05-0.15 Hz           →  FFT tracker strongly recommended
  σ > 0.15 Hz                →  FFT tracker required; investigate motor stability

NOTE ON IMU ACCESS
──────────────────
This script talks to the LSM6DSO32 directly via smbus2 to stay fully
standalone.  If your existing codebase has a tested IMU helper class,
swap `imu_init` / `imu_read_x` for it — the analysis portion is
decoupled from the sampling method.
"""

import os
import sys
import time
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    from smbus2 import SMBus
except ImportError:
    print("ERROR: smbus2 not installed.  Run: pip install smbus2", file=sys.stderr)
    sys.exit(1)

# ─────────────────────────── CONFIG ────────────────────────────
I2C_BUS       = 1
IMU_ADDR      = 0x6A           # LSM6DSO32
FS            = 800            # wall-clock sample rate target (Hz)

# LSM6DSO32 registers
CTRL1_XL      = 0x10
OUTX_L_A      = 0x28

# Accel config: ODR=833Hz (bits [7:4]=0111), FS=±32g (bits [3:2]=11),
# LPF2 disabled (bit [1]=0). Byte = 0b01111100 = 0x7C
CTRL1_XL_VAL  = 0x7C

# Scale: ±32g full-scale → 9.576e-3 m/s² per LSB
ACCEL_SCALE   = 32.0 * 9.80665 / 32768.0

OUT_DIR       = "motor_drift_out"

# ─────────────────────────── IMU ───────────────────────────────
def imu_init(bus):
    bus.write_byte_data(IMU_ADDR, CTRL1_XL, CTRL1_XL_VAL)
    time.sleep(0.05)

def imu_read_x(bus):
    """Read accel X as a float in m/s².  Uses a 2-byte block read."""
    data = bus.read_i2c_block_data(IMU_ADDR, OUTX_L_A, 2)
    raw = (data[1] << 8) | data[0]
    if raw & 0x8000:
        raw -= 0x10000
    return raw * ACCEL_SCALE

# ─────────────────────── ACQUISITION ───────────────────────────
def acquire(total_sec, fs):
    n_total = int(total_sec * fs)
    samples = np.zeros(n_total, dtype=np.float64)
    bus = SMBus(I2C_BUS)
    try:
        imu_init(bus)
        period = 1.0 / fs
        t_start = time.perf_counter()
        next_t = t_start
        dropped = 0
        print(f"Logging {total_sec}s at {fs} Hz (total {n_total} samples)...")
        for i in range(n_total):
            # Busy-wait for next sample slot (keeps jitter low)
            while time.perf_counter() < next_t:
                pass
            try:
                samples[i] = imu_read_x(bus)
            except OSError:
                samples[i] = samples[i-1] if i > 0 else 0.0
                dropped += 1
            next_t += period
            if (i + 1) % (10 * fs) == 0:
                elapsed = time.perf_counter() - t_start
                print(f"  t={elapsed:6.1f}s   {(i+1)/n_total*100:5.1f}%")
        elapsed = time.perf_counter() - t_start
    finally:
        bus.close()
    actual_fs = n_total / elapsed
    print(f"Done.  actual_fs={actual_fs:.2f} Hz  dropped={dropped}")
    if actual_fs < 0.95 * fs:
        print(f"  WARNING: actual sample rate is {actual_fs/fs*100:.1f}% of target.")
        print(f"  Frequency analysis will use actual_fs, not {fs}.")
    return samples, actual_fs, dropped

# ─────────────────────── ANALYSIS ──────────────────────────────
def windowed_peak_analysis(samples, fs, window_sec, f_min, f_max):
    n_win = int(window_sec * fs)
    n_windows = len(samples) // n_win
    hann = np.hanning(n_win)
    freqs_bin = np.fft.rfftfreq(n_win, 1.0 / fs)
    mask = (freqs_bin >= f_min) & (freqs_bin <= f_max)
    band_idx = np.where(mask)[0]
    if band_idx.size == 0:
        raise RuntimeError(f"No FFT bins in [{f_min}, {f_max}] Hz at this window size.")

    t_centers = np.zeros(n_windows)
    f_peaks   = np.zeros(n_windows)
    amp_peaks = np.zeros(n_windows)

    hann_sum = hann.sum()
    for w in range(n_windows):
        seg = samples[w*n_win:(w+1)*n_win].copy()
        seg -= seg.mean()  # remove DC per window
        spec = np.abs(np.fft.rfft(seg * hann))
        k_local = int(np.argmax(spec[mask]))
        k = band_idx[k_local]
        # Parabolic interpolation for sub-bin frequency resolution
        if 0 < k < len(spec) - 1:
            y0, y1, y2 = spec[k-1], spec[k], spec[k+1]
            denom = (y0 - 2*y1 + y2)
            delta = 0.5*(y0 - y2)/denom if denom != 0 else 0.0
        else:
            delta = 0.0
        f_peaks[w]   = (k + delta) * fs / n_win
        amp_peaks[w] = spec[k] * 2.0 / hann_sum  # Hann amplitude correction
        t_centers[w] = (w + 0.5) * window_sec
    return t_centers, f_peaks, amp_peaks, n_windows

def make_recommendation(f_peaks, t_centers, total_sec):
    f_mean = float(np.mean(f_peaks))
    f_std  = float(np.std(f_peaks))
    f_pp   = float(np.max(f_peaks) - np.min(f_peaks))
    if len(t_centers) >= 2:
        slope = float(np.polyfit(t_centers, f_peaks, 1)[0])  # Hz/s
    else:
        slope = 0.0
    drift_total = slope * total_sec

    if f_std < 0.02 and abs(drift_total) < 0.05:
        rec = (f"ROCK-SOLID. Use --manual_freqs {f_mean:.3f} in vss_controller.py. "
               f"Gradient tracker alone is sufficient.")
    elif f_std < 0.05 and abs(drift_total) < 0.15:
        rec = (f"MILD DRIFT. Use --manual_freqs {f_mean:.3f} with µ_omega ≥ 1e-5, "
               f"OR enable FFT tracker for robustness.")
    elif f_std < 0.15:
        rec = ("MODERATE DRIFT. FFT frequency tracker strongly recommended. "
               "Gradient tracker alone will lag and cause beat-frequency artifacts "
               "like you saw in Run 1.")
    else:
        rec = ("UNSTABLE MOTOR. FFT tracker required. Also investigate: "
               "motor power supply ripple, belt tension, thermal state, "
               "Arduino PWM jitter, mechanical coupling.")
    if abs(slope) > 0.002:
        rec += (f"\n  Trend: clear linear drift of {slope*1000:+.3f} mHz/s "
                f"detected — motor is still warming up or cooling. "
                f"Wait longer before controller runs, or re-run this script "
                f"after another 5 min of warm-up.")
    return f_mean, f_std, f_pp, slope, drift_total, rec

# ─────────────────────────── MAIN ──────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0,
                    help="total log duration in seconds (default 300)")
    ap.add_argument("--window", type=float, default=10.0,
                    help="FFT window length in seconds (default 10)")
    ap.add_argument("--fmin", type=float, default=5.0,
                    help="search band lower bound Hz (default 5)")
    ap.add_argument("--fmax", type=float, default=15.0,
                    help="search band upper bound Hz (default 15)")
    ap.add_argument("--no-prompt", action="store_true",
                    help="skip the ENTER-to-start prompt")
    args = ap.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)

    print("motor_drift_char.py")
    print(f"  duration     : {args.duration:.0f} s  ({args.duration/60:.1f} min)")
    print(f"  sample rate  : {FS} Hz")
    print(f"  window       : {args.window:.1f} s ({int(args.window*FS)} samples)")
    print(f"  search band  : [{args.fmin}, {args.fmax}] Hz")
    print()
    print("Ensure the DC motor is running at target speed and has warmed up ~2 min.")
    if not args.no_prompt:
        input("Press ENTER to start logging...")

    samples, actual_fs, dropped = acquire(args.duration, FS)

    print(f"Analyzing {int(args.duration/args.window)} windows of {args.window}s...")
    t_centers, f_peaks, amp_peaks, n_windows = windowed_peak_analysis(
        samples, actual_fs, args.window, args.fmin, args.fmax
    )
    f_mean, f_std, f_pp, slope, drift_total, rec = make_recommendation(
        f_peaks, t_centers, args.duration
    )

    # Save CSV
    csv_path = os.path.join(OUT_DIR, "drift_log.csv")
    with open(csv_path, "w") as f:
        f.write("t_sec,f_peak_hz,amp_peak_m_s2\n")
        for t, fp, ap_ in zip(t_centers, f_peaks, amp_peaks):
            f.write(f"{t:.3f},{fp:.6f},{ap_:.6f}\n")

    # Save summary
    sum_path = os.path.join(OUT_DIR, "summary.txt")
    with open(sum_path, "w") as f:
        f.write("Motor Frequency Drift Characterization\n")
        f.write("======================================\n\n")
        f.write(f"Duration         : {args.duration:.1f} s\n")
        f.write(f"Sample rate      : target {FS} Hz, actual {actual_fs:.2f} Hz\n")
        f.write(f"Window size      : {args.window} s\n")
        f.write(f"Windows analyzed : {n_windows}\n")
        f.write(f"Search band      : [{args.fmin}, {args.fmax}] Hz\n")
        f.write(f"Dropped samples  : {dropped}\n\n")
        f.write("Statistics\n")
        f.write("----------\n")
        f.write(f"  Mean frequency : {f_mean:.4f} Hz\n")
        f.write(f"  Std dev (σ)    : {f_std:.4f} Hz\n")
        f.write(f"  Peak-to-peak   : {f_pp:.4f} Hz\n")
        f.write(f"  Linear drift   : {slope*1000:+.3f} mHz/s "
                f"({drift_total:+.3f} Hz over full run)\n\n")
        f.write("Recommendation\n")
        f.write("--------------\n")
        f.write(rec + "\n")

    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    ax1.plot(t_centers, f_peaks, "o-", color="steelblue", markersize=4)
    ax1.axhline(f_mean, color="gray", linestyle="--", linewidth=0.8,
                label=f"mean={f_mean:.3f} Hz")
    ax1.fill_between(t_centers, f_mean - f_std, f_mean + f_std,
                     alpha=0.15, color="steelblue",
                     label=f"±1σ ({f_std:.3f} Hz)")
    # Trend line if meaningful
    if abs(slope) > 0.001:
        trend = f_mean + slope * (t_centers - t_centers.mean())
        ax1.plot(t_centers, trend, "r--", linewidth=1,
                 label=f"trend {slope*1000:+.2f} mHz/s")
    ax1.set_ylabel("Dominant frequency (Hz)")
    ax1.set_title("Motor Frequency Drift Characterization")
    ax1.legend(loc="best", fontsize=9)
    ax1.grid(alpha=0.3)

    ax2.plot(t_centers, amp_peaks, "o-", color="darkorange", markersize=4)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Peak amplitude (m/s²)")
    ax2.grid(alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(OUT_DIR, "drift_plot.png")
    plt.savefig(plot_path, dpi=120)
    plt.close()

    # Terminal summary
    print()
    print("═" * 62)
    print(f"  Mean frequency : {f_mean:.4f} Hz")
    print(f"  Std dev (σ)    : {f_std:.4f} Hz")
    print(f"  Peak-to-peak   : {f_pp:.4f} Hz")
    print(f"  Linear drift   : {slope*1000:+.3f} mHz/s "
          f"({drift_total:+.3f} Hz over run)")
    print("─" * 62)
    print("  Recommendation:")
    for line in rec.split("\n"):
        print("    " + line)
    print("═" * 62)
    print(f"  CSV     → {csv_path}")
    print(f"  Plot    → {plot_path}")
    print(f"  Summary → {sum_path}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)

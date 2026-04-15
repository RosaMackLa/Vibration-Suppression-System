#!/usr/bin/env python3
"""
plot_plant_phase.py  —  Extract plant phase φ_P from a single_tone_log CSV
──────────────────────────────────────────────────────────────────────────────
Loads the CSV produced by single_tone_log.py, fits sinusoids to both the
commanded position and the measured acceleration, and reports the true
secondary-path phase φ_P.

USAGE
─────
  python3 plot_plant_phase.py single_tone_out/log_YYYYMMDD_HHMMSS.csv

  Optional:
    --trim_start  seconds to skip from the start of the log (default 1.0)
                  (removes any remaining transient after the settle window)
    --trim_end    seconds to skip from the end   (default 0.5)

OUTPUT
──────
  Prints φ_P (degrees) and |G| (m/s² per mm).
  Saves a PNG plot alongside the CSV.

CONCEPTUAL NOTE ON WHAT WE'RE MEASURING
────────────────────────────────────────
The secondary path maps:
    carriage displacement x_cmd(t) [mm]  →  base-plate acceleration a(t) [m/s²]

If we command:
    x_cmd(t) = A · sin(ω·t)

And the plant has gain G and phase shift φ_P at this frequency, then:
    a(t) ≈ G·A · sin(ω·t + φ_P)

So:
    φ_P  = phase_of(a)  −  phase_of(x_cmd)

This is the value FxLMS uses in its secondary-path filter.  It is NOT the
same as the kinematic ω² phase (which would give exactly −180° for a rigid
inertial system).  The structural dynamics add their own phase on top.

Why fit a sinusoid rather than just take the FFT peak phase?
─────────────────────────────────────────────────────────────
FFT phase is sensitive to the exact sample count and windowing.  A least-
squares sine fit uses ALL the data points and is naturally robust to noise.
It also lets us extract an amplitude (for G) in the same step.
"""

import argparse
import csv
import os
import sys
import math

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ═══════════════════════════════════════════════════════════════════════════
# CSV reader — parses the header comment lines for run parameters
# ═══════════════════════════════════════════════════════════════════════════

def load_csv(path: str):
    params = {}
    rows   = []

    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if row[0].startswith("#"):
                key = row[0].lstrip("# ").strip()
                val = float(row[1])
                params[key] = val
            elif row[0] == "t_s":
                continue   # skip column header
            else:
                rows.append((float(row[0]), float(row[1])))

    t = np.array([r[0] for r in rows])
    a = np.array([r[1] for r in rows])
    return params, t, a

# ═══════════════════════════════════════════════════════════════════════════
# Least-squares sine fit
# ═══════════════════════════════════════════════════════════════════════════

def sine_fit(t: np.ndarray, y: np.ndarray, f: float):
    """
    Fit  y(t) ≈ A·cos(ω·t) + B·sin(ω·t) + C  (offset C)
    using linear least squares.

    This is equivalent to fitting  R·sin(ω·t + φ)  but with no nonlinear
    solver — it's a direct matrix solve.  Very robust to noise.

    Returns (amplitude, phase_deg, offset, residual_rms)
    where  amplitude = sqrt(A² + B²)
    and    phase_deg = atan2(A, B)  in degrees

    Why atan2(A, B) and not atan2(B, A)?
    ──────────────────────────────────────
    We write: R·sin(ω·t + φ) = R·cos(φ)·sin(ω·t) + R·sin(φ)·cos(ω·t)
                              = B·sin(ω·t)         + A·cos(ω·t)
    So A = R·sin(φ)  →  φ = atan2(A, B).
    """
    omega = 2.0 * math.pi * f
    C_col = np.cos(omega * t)
    S_col = np.sin(omega * t)
    O_col = np.ones_like(t)

    # Build design matrix [cos, sin, 1]
    X = np.column_stack([C_col, S_col, O_col])
    # Solve least squares: X @ [A, B, C]' ≈ y
    coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    A, B, offset = coeffs

    amplitude = math.sqrt(A**2 + B**2)
    phase_deg = math.degrees(math.atan2(A, B))

    residual_rms = float(np.sqrt(np.mean((y - X @ coeffs)**2)))
    return amplitude, phase_deg, offset, residual_rms

# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(description="Plot and quantify plant phase φ_P")
    p.add_argument("csv_path",    type=str)
    p.add_argument("--trim_start", type=float, default=1.0,
                   help="Skip this many seconds from start of log (default 1.0)")
    p.add_argument("--trim_end",   type=float, default=0.5,
                   help="Skip this many seconds from end of log (default 0.5)")
    return p.parse_args()


def main():
    args   = parse_args()
    params, t_raw, a_raw = load_csv(args.csv_path)

    freq    = params.get("freq_hz",    8.5)
    amp_mm  = params.get("amp_mm",     5.0)
    omega   = 2.0 * math.pi * freq

    print(f"Loaded {len(t_raw)} samples from {args.csv_path}")
    print(f"  freq={freq} Hz  amp={amp_mm} mm")
    print(f"  log span: {t_raw[0]:.2f} s → {t_raw[-1]:.2f} s")

    # ── Trim ──────────────────────────────────────────────────────────────
    t0 = t_raw[0] + args.trim_start
    t1 = t_raw[-1] - args.trim_end
    if t1 <= t0:
        sys.exit("ERROR: After trimming, no data remains.  Reduce --trim_start or --trim_end.")

    mask = (t_raw >= t0) & (t_raw <= t1)
    t    = t_raw[mask]
    a    = a_raw[mask]
    n    = len(t)
    span = t[-1] - t[0]
    fs   = n / span

    print(f"  After trim: {n} samples over {span:.2f} s  (fs ≈ {fs:.0f} Hz)")

    # ── Commanded position (analytical — no noise) ─────────────────────
    # t is referenced from when the first wave fired, so x_cmd(t) = A·sin(ω·t)
    x_cmd = amp_mm * np.sin(omega * t)   # mm

    # ── Sine fits ──────────────────────────────────────────────────────
    # Position reference (should give phase ≈ 0°, amplitude ≈ amp_mm)
    amp_x,  phi_x,  _,       res_x = sine_fit(t, x_cmd, freq)
    # Measured acceleration
    amp_a,  phi_a,  offset_a, res_a = sine_fit(t, a,     freq)

    # ── Plant phase and gain ───────────────────────────────────────────
    phi_P = phi_a - phi_x     # degrees

    # Wrap to (-180, 180]
    while phi_P >  180: phi_P -= 360
    while phi_P < -180: phi_P += 360

    # Plant gain: m/s² response per mm input
    G = amp_a / amp_mm   # units: (m/s²)/mm = s⁻²

    # Signal-to-noise: compare fit amplitude to residual RMS
    snr_a_db = 20 * math.log10(amp_a / res_a) if res_a > 0 else float("inf")

    print()
    print("═══ RESULTS ════════════════════════════════════════════════")
    print(f"  Plant phase  φ_P = {phi_P:+.1f}°")
    print(f"  Plant gain   |G| = {G:.4f}  (m/s²)/mm")
    print(f"  IMU fit SNR      = {snr_a_db:.1f} dB  (>6 dB is trustworthy)")
    print(f"  IMU DC offset    = {offset_a:.3f} m/s²  (gravity+tilt — normal)")
    print(f"  Accel amplitude  = {amp_a:.3f} m/s²")
    print(f"  Fit residual rms = {res_a:.3f} m/s²")
    print("════════════════════════════════════════════════════════════")

    if snr_a_db < 6:
        print()
        print("  ⚠  SNR < 6 dB — phase measurement is unreliable.")
        print("     Try increasing --amp_mm (target 10–16 mm) and")
        print("     confirm DC motor is fully off and mechanically stopped.")

    # ── Plot ───────────────────────────────────────────────────────────
    # Show only a 3-cycle window for clarity
    n_cycles_show = 3
    t_show_dur = n_cycles_show / freq
    t_plot_end = t[0] + t_show_dur
    pm = t <= t_plot_end
    t_p  = t[pm]
    a_p  = a[pm]
    x_p  = x_cmd[pm]

    # Build fit curves over the plot window (smooth)
    t_fine = np.linspace(t_p[0], t_p[-1], 1000)
    # Reconstruct fit from amplitude and phase
    A_x    = amp_x * np.sin(omega * t_fine + math.radians(phi_x))
    A_a    = amp_a * np.sin(omega * t_fine + math.radians(phi_a))
    # Normalise position to mm scale for overlay
    # (already in mm — no scaling needed)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(
        f"Plant Phase Measurement — {freq} Hz, {amp_mm} mm\n"
        f"φ_P = {phi_P:+.1f}°   |G| = {G:.4f} (m/s²)/mm   SNR = {snr_a_db:.1f} dB",
        fontsize=12
    )

    # Panel 1: Commanded position
    ax1 = axes[0]
    ax1.plot(t_p - t[0], x_p, color="steelblue", linewidth=1.2, label="x_cmd (mm)")
    ax1.plot(t_fine - t[0], A_x, "r--", linewidth=1.0, label=f"fit (φ={phi_x:.1f}°)")
    ax1.set_ylabel("Commanded\nposition (mm)")
    ax1.axhline(0, color="gray", linewidth=0.5)
    ax1.legend(fontsize=8, loc="upper right")
    ax1.grid(True, alpha=0.3)

    # Panel 2: Measured acceleration
    ax2 = axes[1]
    ax2.plot(t_p - t[0], a_p, color="darkorange", linewidth=0.8, alpha=0.7, label="IMU accel (m/s²)")
    ax2.plot(t_fine - t[0], A_a, "r--", linewidth=1.0,
             label=f"fit  A={amp_a:.3f} m/s²  φ={phi_a:.1f}°")
    ax2.set_ylabel("Base-plate\nacceleration (m/s²)")
    ax2.axhline(offset_a, color="gray", linewidth=0.5, linestyle=":")
    ax2.legend(fontsize=8, loc="upper right")
    ax2.grid(True, alpha=0.3)

    # Panel 3: Overlay (normalised) to show phase shift visually
    ax3 = axes[2]
    x_norm = x_p / max(abs(x_p)) if max(abs(x_p)) > 0 else x_p
    a_zero = a_p - offset_a
    a_norm = a_zero / max(abs(a_zero)) if max(abs(a_zero)) > 0 else a_zero
    ax3.plot(t_p - t[0], x_norm, color="steelblue",   linewidth=1.2, label="x_cmd (normalised)")
    ax3.plot(t_p - t[0], a_norm, color="darkorange",  linewidth=1.2, label="accel (normalised, DC removed)")
    ax3.set_xlabel("Time since log start (s)")
    ax3.set_ylabel("Normalised\namplitude")
    ax3.legend(fontsize=8, loc="upper right")
    ax3.grid(True, alpha=0.3)
    ax3.set_title(f"Phase lag: φ_P = {phi_P:+.1f}°  (positive = accel leads, negative = accel lags)",
                  fontsize=9)

    plt.tight_layout()

    out_png = args.csv_path.replace(".csv", "_phase_plot.png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"\nPlot saved → {out_png}")

if __name__ == "__main__":
    main()

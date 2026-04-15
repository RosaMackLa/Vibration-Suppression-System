#!/usr/bin/env python3
"""
plant_phase_measure.py  —  Empirical plant phase / gain measurement
═══════════════════════════════════════════════════════════════════

PURPOSE
───────
Directly measure the secondary-path plant transfer function at a single
operating frequency, using the same DMA waveform pipeline the controller
uses.  The DC disturbance motor MUST be off — we measure how the IMU
responds to clean, known stepper motion.

WHY THIS EXISTS
───────────────
Every previous plant_id.py run had deeply negative SNR because the
disturbance motor was running and dwarfing the stepper signal.  As a
result, G and φ_P have never been trustworthy.  The LMS controller
needs φ_P to within ±90° to converge in the correct direction; without
it, the LMS happily locks onto a (C, S) that minimises its internal
model but doesn't actually reduce vibration at the IMU.  This script
fixes that by removing the disturbance, driving a clean sinusoid, and
fitting the IMU response with no noise floor competition.

METHOD
──────
1. Drive stepper as x(t) = A·cos(2π·f·t)  for `--duration` seconds.
2. In parallel, sample the IMU at 800 Hz.
3. Wait `--settle` seconds, then keep the rest as the measurement window.
4. Demodulate IMU(t) at frequency f using a least-squares sine fit:
       IMU(t) ≈ I·cos(2π·f·t) + Q·sin(2π·f·t) + DC
   The plant maps commanded displacement to measured acceleration:
       G  = sqrt(I² + Q²) / A_mm        [(m/s²) / mm]
       φ_P = atan2(Q, I)                [rad — phase IMU leads command]

DROP-IN COMPATIBILITY
─────────────────────
Imports compute_chunk, k_steps_per_mm, hw_read_accel from vss_controller
so phase frame, pulse generation, and IMU access exactly match the
controller.  Whatever phase relationship the controller sees, this
script sees too.

USAGE
─────
  # Make sure DC disturbance motor is OFF.
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 plant_phase_measure.py \
      --freq 8.5 --amp 5.0 --duration 30 --settle 3

  # To sweep multiple frequencies (one run per freq):
  for f in 8.0 8.5 9.0; do
      sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 plant_phase_measure.py \
          --freq $f --amp 5.0 --duration 30 --settle 3 \
          --out plant_${f}Hz.csv
  done

OUTPUTS
───────
  plant_phase_out/
    plant_<f>Hz.csv  — time-series of (t, imu, x_cmd) over measurement window
    plant_<f>Hz.png  — overlay plot: IMU vs commanded x, with fit
    summary printed to terminal:  G, φ_P (deg), SNR, recommended controller flag

INTERPRETATION GUIDE
────────────────────
  φ_P near 0°            → "perfect" plant; cancellation in opposition with
                             disturbance.  LMS converges with no offset.
  φ_P near ±180°         → sign inversion only.  Equivalent to flipping
                             --dir_invert; LMS converges either way once
                             that's right.
  φ_P near ±90°          → quadrature plant.  LMS converges to a (C, S)
                             that's orthogonal to optimal — produces motion
                             but doesn't reduce error.  This is the failure
                             mode the recent runs are showing.  Fix is to
                             pre-rotate the LMS reference by φ_P.
  Anything in between    → standard FxLMS pre-rotation handles it cleanly.
"""

import argparse
import math
import os
import sys
import time
import threading

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pigpio

# Reuse the controller's primitives so we measure exactly what the
# controller experiences — same waveform engine, same IMU path, same
# kinematics.
from vss_controller import (
    compute_chunk,
    wave_dur_s,
    k_steps_per_mm,
    hw_read_accel,
    DEFAULT_PULLEY_D_MM,
    DEFAULT_PULSES_PER_REV,
    HALF_TRAVEL_MM,
    DEFAULT_STEP_BCM,
    DEFAULT_DIR_BCM,
    ENABLE_PIN,
)

OUT_DIR = "plant_phase_out"


# ─────────────────────────── IMU sampling thread ────────────────────────────
def imu_sampler(samples_t, samples_v, fs, running, t_wall_start):
    """Sample IMU at fs Hz into preallocated arrays."""
    period = 1.0 / fs
    next_t = time.monotonic()
    i = 0
    n_max = len(samples_v)
    while running[0] and i < n_max:
        now = time.monotonic()
        if now < next_t:
            time.sleep(max(0.0, next_t - now))
            now = time.monotonic()
        try:
            samples_v[i] = hw_read_accel()
        except Exception:
            samples_v[i] = samples_v[i - 1] if i > 0 else 0.0
        samples_t[i] = now - t_wall_start
        i += 1
        next_t += period
    return i


# ─────────────────────────── Stepper drive thread ───────────────────────────
def stepper_driver(pi, args, k, running, F_init, t_wall_start):
    """
    Drive stepper at single frequency f, amplitude A, using compute_chunk.
    Mirrors the controller's double-buffer DMA loop so phase frame matches.
    """
    chunk_s = args.chunk_ms / 1000.0
    A_list = [args.amp]
    F_list = [args.freq]
    PHI    = [0.0]
    step_phis = [0.0]   # persistent phase accumulator — same as controller

    # ── First chunk ──
    pulses_c, last_dir, _, _ = compute_chunk(
        0.0, chunk_s, A_list, F_list, PHI, k,
        args.max_sps, args.deadband_sps,
        args.step, args.dir, 1, args.dir_invert,
        args.dir_blanking_us, args.dir_setup_us,
        step_phis,
    )
    dur_c = wave_dur_s(pulses_c) or chunk_s
    pi.wave_add_generic(pulses_c)
    wave_c = pi.wave_create()
    pi.wave_send_once(wave_c)
    t_chunk_wall = time.monotonic()

    s_est = 0.0

    while running[0]:
        pulses_n, dir_n, _, steps_n = compute_chunk(
            0.0, chunk_s, A_list, F_list, PHI, k,
            args.max_sps, args.deadband_sps,
            args.step, args.dir, last_dir, args.dir_invert,
            args.dir_blanking_us, args.dir_setup_us,
            step_phis,
        )
        dur_n = wave_dur_s(pulses_n) or chunk_s

        sleep_s = dur_c - (time.monotonic() - t_chunk_wall) - 0.010
        if sleep_s > 0:
            time.sleep(sleep_s)
        while pi.wave_tx_busy() and running[0]:
            time.sleep(0.0001)

        s_est += steps_n
        x_est = s_est / k
        if abs(x_est) > HALF_TRAVEL_MM + 1.0:
            print(f"\n[stepper] TRAVEL SAFETY: x_est={x_est:.1f}mm — stopping.")
            running[0] = False
            break

        pi.wave_delete(wave_c)
        pi.wave_add_generic(pulses_n)
        wave_n = pi.wave_create()
        pi.wave_send_once(wave_n)
        t_chunk_wall = time.monotonic()

        wave_c   = wave_n
        dur_c    = dur_n
        last_dir = dir_n


# ─────────────────────────── Sine fit & analysis ────────────────────────────
def sine_fit(t, y, f):
    """
    Least-squares fit y(t) ≈ I·cos(2π·f·t) + Q·sin(2π·f·t) + DC.
    Returns (I, Q, DC, residual_rms).
    """
    omega = 2.0 * math.pi * f
    A_mat = np.column_stack([np.cos(omega * t), np.sin(omega * t), np.ones_like(t)])
    coeffs, *_ = np.linalg.lstsq(A_mat, y, rcond=None)
    I, Q, DC = coeffs
    fit = I * np.cos(omega * t) + Q * np.sin(omega * t) + DC
    resid = y - fit
    return I, Q, DC, float(np.sqrt(np.mean(resid ** 2)))


def interpret_phi_p(phi_p_deg):
    """Return (regime_label, action_recommendation)."""
    a = abs(phi_p_deg)
    if a < 30:
        return ("near-0°  (in-phase plant)",
                "LMS should converge directly. No pre-rotation needed.")
    if a > 150:
        return ("near-±180°  (sign-inverted plant)",
                "Equivalent to flipping --dir_invert. Try the LMS run with "
                "--dir_invert toggled to its opposite of what you've been using.")
    if 60 <= a <= 120:
        return ("near-±90°  (QUADRATURE plant — this is the failure mode)",
                f"LMS converges orthogonal to optimal — produces motion but "
                f"doesn't reduce error.  FIX: pre-rotate LMS reference by "
                f"φ_P = {phi_p_deg:+.1f}° (cos_ref = cos(phi - φ_P), "
                f"sin_ref = sin(phi - φ_P)).")
    return (f"intermediate ({phi_p_deg:+.1f}°)",
            f"Pre-rotate LMS reference by φ_P = {phi_p_deg:+.1f}° "
            f"in lms_thread_fn for clean convergence.")


# ─────────────────────────────── Main ───────────────────────────────────────
def main():
    p = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--freq",     type=float, required=True, help="Drive frequency [Hz]")
    p.add_argument("--amp",      type=float, default=5.0,   help="Drive amplitude [mm]")
    p.add_argument("--duration", type=float, default=30.0,  help="Total drive duration [s]")
    p.add_argument("--settle",   type=float, default=3.0,   help="Initial seconds to discard [s]")
    p.add_argument("--imu_fs",   type=float, default=800.0, help="IMU sample rate [Hz]")

    # Mirror controller hardware args so phase frame matches exactly
    p.add_argument("--step",            type=int,   default=DEFAULT_STEP_BCM)
    p.add_argument("--dir",             type=int,   default=DEFAULT_DIR_BCM)
    p.add_argument("--dir_invert",      action="store_true",
                   help="MUST match how you'll run the controller. If you've been "
                        "running the controller with --dir_invert, use it here too.")
    p.add_argument("--pulley_d_mm",     type=float, default=DEFAULT_PULLEY_D_MM)
    p.add_argument("--pulses_per_rev",  type=float, default=DEFAULT_PULSES_PER_REV)
    p.add_argument("--max_sps",         type=float, default=50000.0)
    p.add_argument("--deadband_sps",    type=float, default=20.0)
    p.add_argument("--chunk_ms",        type=float, default=50.0)
    p.add_argument("--dir_setup_us",    type=int,   default=20)
    p.add_argument("--dir_blanking_us", type=int,   default=200)

    p.add_argument("--out", type=str, default=None,
                   help="CSV output filename (default: plant_<f>Hz.csv)")
    p.add_argument("--no_prompt", action="store_true",
                   help="Skip the 'is the motor off?' prompt")
    args = p.parse_args()

    if args.amp > 0.7 * HALF_TRAVEL_MM:
        sys.exit(f"ERROR: --amp {args.amp} mm too close to ±{HALF_TRAVEL_MM} mm "
                 f"limit; reduce to ≤ {0.7 * HALF_TRAVEL_MM:.0f} mm.")

    os.makedirs(OUT_DIR, exist_ok=True)
    out_csv = args.out or f"plant_{args.freq:.2f}Hz.csv"
    out_csv = os.path.join(OUT_DIR, out_csv)
    out_png = out_csv.replace(".csv", ".png")

    k = k_steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    print("═" * 60)
    print("  PLANT PHASE / GAIN MEASUREMENT")
    print("═" * 60)
    print(f"  Frequency  : {args.freq:.3f} Hz")
    print(f"  Amplitude  : {args.amp:.2f} mm")
    print(f"  Duration   : {args.duration:.1f} s   (settle={args.settle:.1f}s, measure={args.duration - args.settle:.1f}s)")
    print(f"  steps/mm   : {k:.4f}")
    print(f"  dir_invert : {args.dir_invert}")
    print()
    print("  ⚠  DC DISTURBANCE MOTOR MUST BE OFF.")
    print("     If the motor is running, the result will be contaminated.")
    if not args.no_prompt:
        input("  Press ENTER once the motor is OFF and the carriage is centred...")

    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("ERROR: pigpiod not running.  sudo systemctl start pigpiod")

    pi.set_mode(args.dir,   pigpio.OUTPUT)
    pi.set_mode(args.step,  pigpio.OUTPUT)
    pi.set_mode(ENABLE_PIN, pigpio.OUTPUT)
    pi.write(ENABLE_PIN, 1)
    pi.wave_clear()

    # Preallocate IMU buffer
    n_total = int(args.duration * args.imu_fs) + 100
    samples_t = np.zeros(n_total, dtype=np.float64)
    samples_v = np.zeros(n_total, dtype=np.float64)

    running = [True]
    t_wall_start = time.monotonic()

    # ── Spawn stepper driver thread ──
    stepper_t = threading.Thread(
        target=stepper_driver,
        args=(pi, args, k, running, [args.freq], t_wall_start),
        daemon=True,
        name="stepper",
    )
    stepper_t.start()

    # ── Sample IMU on main thread (priority over Python thread switching) ──
    print(f"\n  Sampling IMU at {args.imu_fs:.0f} Hz for {args.duration:.1f} s...")
    n_samples = imu_sampler(samples_t, samples_v, args.imu_fs, running, t_wall_start)

    # Stop stepper cleanly
    print("  Stopping stepper...")
    running[0] = False
    stepper_t.join(timeout=2.0)
    try:
        pi.wave_tx_stop()
        pi.wave_clear()
        pi.write(args.step, 0)
        pi.write(args.dir,  0)
        pi.write(ENABLE_PIN, 0)
    except Exception:
        pass
    pi.stop()

    # ── Trim to measurement window ──
    t_arr = samples_t[:n_samples]
    v_arr = samples_v[:n_samples]
    mask = t_arr >= args.settle
    if mask.sum() < 100:
        sys.exit("ERROR: not enough samples after settle window.")
    t_meas = t_arr[mask] - args.settle    # zero-base for fit (matches stepper t=0 frame)
    v_meas = v_arr[mask]

    # NOTE on phase frame: compute_chunk's per-tone step_phis starts at 0
    # at the very first call (t=0 of stepper_driver, which equals
    # t_wall_start of this script).  After the settle window the stepper
    # phase has accumulated 2π·f·t_settle; subtracting `args.settle` from
    # the IMU timestamps re-zeros the fit's reference, BUT the stepper
    # has its own continuous accumulator.  To avoid a constant phase
    # offset that confuses interpretation, fit at the actual frequency
    # using IMU times still relative to t_wall_start.
    t_meas_abs = t_arr[mask]   # use absolute time for fit so frame matches stepper

    print(f"  Got {len(t_meas_abs)} samples in measurement window "
          f"({t_meas_abs[0]:.2f}s → {t_meas_abs[-1]:.2f}s)")

    # ── Sine fit ──
    I, Q, DC, resid_rms = sine_fit(t_meas_abs, v_meas, args.freq)
    A_meas = math.sqrt(I ** 2 + Q ** 2)            # m/s² @ freq
    phi_p  = math.atan2(Q, I)                      # rad
    phi_p_deg = math.degrees(phi_p)

    # Plant gain: (m/s² per mm of commanded displacement)
    G = A_meas / args.amp

    # SNR: signal power vs residual power
    sig_power = 0.5 * A_meas ** 2
    noise_power = resid_rms ** 2
    snr_db = 10.0 * math.log10(sig_power / noise_power) if noise_power > 0 else float("inf")

    regime, action = interpret_phi_p(phi_p_deg)

    # ── Print results ──
    print()
    print("─" * 60)
    print("  RESULTS")
    print("─" * 60)
    print(f"  IMU response amplitude : {A_meas * 1000:.2f} mm/s²")
    print(f"  Residual RMS           : {resid_rms * 1000:.2f} mm/s²")
    print(f"  SNR                    : {snr_db:+.1f} dB")
    print(f"  Plant gain G           : {G:.4f} (m/s²)/mm")
    print(f"  Plant phase φ_P        : {phi_p_deg:+.1f} °")
    print()
    print(f"  Regime: {regime}")
    print(f"  Action: {action}")
    print("─" * 60)

    if snr_db < 10:
        print("  ⚠  SNR is low.  Increase --amp or --duration, or check that the")
        print("     disturbance motor is truly off.  Below ~10 dB, φ_P estimate")
        print("     has ±20° uncertainty or worse.")

    # ── Save CSV ──
    # Reconstruct the commanded displacement at IMU sample times for the plot.
    omega = 2.0 * math.pi * args.freq
    x_cmd = args.amp * np.cos(omega * t_meas_abs)
    fit   = I * np.cos(omega * t_meas_abs) + Q * np.sin(omega * t_meas_abs) + DC

    with open(out_csv, "w") as f:
        f.write("t_sec,imu_m_s2,x_cmd_mm,fit_m_s2\n")
        for ti, vi, xi, fi in zip(t_meas_abs, v_meas, x_cmd, fit):
            f.write(f"{ti:.4f},{vi:.6f},{xi:.4f},{fi:.6f}\n")
    print(f"  CSV  → {out_csv}")

    # ── Plot: IMU vs commanded, plus fit overlay ──
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)

    # Top: commanded displacement vs IMU acceleration (different y-axes)
    ax1 = axes[0]
    ax1.plot(t_meas_abs, x_cmd, color="steelblue", linewidth=1.0,
             label=f"x_cmd  ({args.amp:.1f} mm @ {args.freq:.2f} Hz)")
    ax1.set_ylabel("Commanded displacement [mm]", color="steelblue")
    ax1.tick_params(axis="y", labelcolor="steelblue")
    ax1.grid(True, alpha=0.3)
    ax1b = ax1.twinx()
    ax1b.plot(t_meas_abs, v_meas, color="darkorange", linewidth=0.6, alpha=0.6,
              label="IMU (raw)")
    ax1b.plot(t_meas_abs, fit, color="crimson", linewidth=1.2,
              label=f"fit ({A_meas*1000:.1f} mm/s², φ_P={phi_p_deg:+.1f}°)")
    ax1b.set_ylabel("IMU acceleration [m/s²]", color="darkorange")
    ax1b.tick_params(axis="y", labelcolor="darkorange")
    ax1b.legend(loc="upper right", fontsize=8)
    ax1.set_title(f"Plant measurement @ {args.freq:.3f} Hz  —  "
                  f"G={G:.3f} (m/s²)/mm,  φ_P={phi_p_deg:+.1f}°,  SNR={snr_db:+.1f} dB")

    # Bottom: residual (IMU - fit) — should be ~white noise if model is good
    ax2 = axes[1]
    ax2.plot(t_meas_abs, v_meas - fit, color="gray", linewidth=0.6)
    ax2.axhline(0, color="black", linewidth=0.5)
    ax2.set_ylabel("Residual [m/s²]")
    ax2.set_xlabel("Time [s]")
    ax2.set_title(f"Fit residual (RMS = {resid_rms * 1000:.2f} mm/s²)")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f"  Plot → {out_png}")
    print()
    print("Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)

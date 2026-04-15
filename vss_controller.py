#!/usr/bin/env python3
"""
vss_controller.py  —  VSS closed-loop adaptive vibration suppression

PIPELINE
────────
  PHASE 1 — IDENTIFY : sample IMU for --id_dur seconds, FFT, pick top ≤N tones
  PHASE 2 — CANCEL   : DMA waveform stepper at identified frequencies;
                        LMS thread continuously adapts (C, S, ω) coefficients
                        to minimise residual IMU acceleration

CONTROL LAW (per tone i, ωᵢ = 2πfᵢ)
──────────────────────────────────────
  Cancellation signal : x(t) = Σ [Cᵢ·cos(φᵢ) + Sᵢ·sin(φᵢ)]   [mm]
                        where φᵢ is a continuously accumulated phase
  Error               : e(t) = IMU x-axis acceleration            [m/s²]
  LMS updates         : Cᵢ  -= µ      · e(t) · cos(φᵢ)
                        Sᵢ  -= µ      · e(t) · sin(φᵢ)
                        ωᵢ  -= µ_omega · e(t) · (Sᵢ·cos(φᵢ) − Cᵢ·sin(φᵢ))

  The ω update uses the quadrature signal (S·cos − C·sin), which is the
  component of x(t) that is 90° ahead of the current output and points in
  the direction of increasing frequency.  This is an approximation of
  ∂x/∂ω that drops the growing t factor, absorbed into µ_omega instead.

  A phase accumulator (φᵢ += ωᵢ·dt each sample) replaces ωᵢ·t so that
  when ω changes the phase evolves continuously with no discontinuity.

  µ_omega must be much smaller than µ: frequency is a global parameter
  and wrong updates corrupt both C and S simultaneously.  Start at µ·0.01.

ARCHITECTURE
────────────
  Main thread   : DMA double-buffer loop — reads latest (C, S, ω) at every
                  chunk boundary and passes adapted frequency to compute_chunk
  LMS thread    : ~control_fs Hz — IMU read → gradient update → clamp → log;
                  also pushes IMU samples into a ring buffer for the FFT thread
  FFT thread    : --fft_tracker only — every --fft_update_sec, computes a
                  Hann-windowed FFT over the ring buffer, finds the dominant
                  peak in [min_freq, max_freq] with parabolic interpolation,
                  and writes the result into the shared frequency state.
                  When the FFT tracker is active, the gradient ω update in
                  the LMS thread is automatically disabled to maintain a
                  single-writer invariant on the frequency state.
  Status thread : ~status_hz Hz print (shows tracked frequency) + keepalive

PHASE FRAME — historical fix
────────────────────────────
  Earlier versions of compute_chunk used absolute time `t` for stepper
  phase: cos(2π·F·t + PHI).  This created a phase discontinuity of
  2π·ΔF·t at every chunk boundary where ω changed — manageable for the
  gradient tracker's microscopic ω updates, catastrophic for the FFT
  tracker (a 0.05 Hz step at t=60s = 6 full rotations of misalignment).
  Fixed: compute_chunk now maintains a per-tone phase accumulator
  (step_phis) that evolves identically to the LMS phi accumulator.
  When ω changes, evolution rate changes but accumulated phase does not.

AMPLITUDE CLAMPING
──────────────────
  Each tone has its own per-tone amplitude limit derived from --max_amp_per_tone
  and the per-tone sps constraint (peak_sps = A × 2πf × k ≤ max_sps).
  These are independent physical constraints — a high-frequency tone hits the
  sps wall at a lower amplitude than a low-frequency tone.  per_tone_max_amp
  is computed in main() and threaded through to clamp_coeffs so that the 9.3 Hz
  tone is not penalised by the 28 Hz tone's tighter sps limit.

LOGGING & PLOTTING
──────────────────
  LMS state (t, e, C, S, f) is logged at --log_hz (default 100 Hz) into
  in-memory lists.  After cancellation stops, plot_run() generates a
  4-panel convergence plot (identical to vss_lms_sim.py) saved to --plot_out.
  Use --no_plot to skip.  Use --plot_out run1.png to keep runs distinct.

USAGE
─────
  # Real hardware (Pi 4B with DM556 + LSM6DSO):
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 vss_controller.py \
      --id_dur 30 --dir_invert --max_sps 50000

  # Single known tone, fast µ:
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 vss_controller.py \
      --skip_id --manual_freqs 9.3 --mu 1e-3 --mu_omega 0 \
      --max_sps 50000 --max_amp_per_tone 9 --cancel_dur 120 --dir_invert

  # Simulation (any machine):
  python3 vss_controller.py --simulate --id_dur 5 --mu 5e-3

  # Adaptive frequency tracking enabled:
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 vss_controller.py \
      --id_dur 30 --dir_invert --max_sps 50000 --mu 5e-4 --mu_omega 5e-6
"""

import argparse
import math
import time
import threading
import sys
import random
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('Agg')   # headless — no display needed on Pi
import matplotlib.pyplot as plt
import pigpio

# ── Optional hardware IMU import ───────────────────────────────────────────────
try:
    import busio
    import board
    from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32
    _HW_IMU_AVAILABLE = True
except ImportError:
    _HW_IMU_AVAILABLE = False

# ══════════════════════════════════════════════════════════════════════════════
# Constants
# ══════════════════════════════════════════════════════════════════════════════

DEFAULT_PULLEY_D_MM    = 11.84
DEFAULT_PULSES_PER_REV = 3200
HALF_TRAVEL_MM         = 75.0
DEFAULT_STEP_BCM       = 13        # PWM1 — avoids snd_bcm2835 conflict
DEFAULT_DIR_BCM        = 16
PULSE_HIGH_US          = 5
ENABLE_PIN             = 25

DEFAULT_SIM_PLANT_GAIN = 0.01      # m/s² per mm of counterweight displacement

# ══════════════════════════════════════════════════════════════════════════════
# IMU helpers
# ══════════════════════════════════════════════════════════════════════════════

_imu_singleton = None

def _get_imu():
    global _imu_singleton
    if _imu_singleton is None:
        _imu_singleton = LSM6DSO32(busio.I2C(board.SCL, board.SDA))
    return _imu_singleton


def hw_read_accel() -> float:
    """Read x-axis acceleration from LSM6DSO [m/s²]."""
    return _get_imu().acceleration[0]


def sim_disturbance(t: float) -> float:
    """
    Synthetic disturbance: two tones sized so the VSS can fully cancel them
    within its ±75mm / 60mm-total-amplitude limits with plant_gain=0.01.
      5 Hz  @ 0.4 m/s²  → needs 40 mm counterweight amplitude to cancel
     12 Hz  @ 0.15 m/s² → needs 15 mm counterweight amplitude to cancel
      total 55 mm < 60 mm cap  ✓
    """
    return (0.40 * math.sin(2 * math.pi *  5.0 * t) +
            0.15 * math.sin(2 * math.pi * 12.0 * t) +
            0.003 * (2 * random.random() - 1))


def sim_read_accel(t: float, lms_coeffs: list, freqs: list,
                   plant_gain: float) -> float:
    """
    Simulated IMU = disturbance + effect of counterweight motion.
    Plant model (linear, no phase lag): effect = plant_gain × Σ xᵢ(t)
    """
    e = sim_disturbance(t)
    for i, f in enumerate(freqs):
        wt = 2.0 * math.pi * f * t
        C, S = lms_coeffs[i]
        e += plant_gain * (C * math.cos(wt) + S * math.sin(wt))
    return e

# ══════════════════════════════════════════════════════════════════════════════
# Kinematics
# ══════════════════════════════════════════════════════════════════════════════

def k_steps_per_mm(pulley_d_mm: float, pulses_per_rev: float) -> float:
    return pulses_per_rev / (math.pi * pulley_d_mm)

# ══════════════════════════════════════════════════════════════════════════════
# Argument parsing
# ══════════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(
        description="VSS closed-loop controller — identify dominant vibration "
                    "tones, then adaptively cancel them.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # ── Hardware ──────────────────────────────────────────────────────────────
    hw = p.add_argument_group("Hardware")
    hw.add_argument("--step",            type=int,   default=DEFAULT_STEP_BCM)
    hw.add_argument("--dir",             type=int,   default=DEFAULT_DIR_BCM)
    hw.add_argument("--dir_invert",      action="store_true")
    hw.add_argument("--pulley_d_mm",     type=float, default=DEFAULT_PULLEY_D_MM)
    hw.add_argument("--pulses_per_rev",  type=float, default=DEFAULT_PULSES_PER_REV)
    hw.add_argument("--max_sps",         type=float, default=8000.0)
    hw.add_argument("--deadband_sps",    type=float, default=20.0)
    hw.add_argument("--chunk_ms",        type=float, default=50.0)
    hw.add_argument("--dir_setup_us",    type=int,   default=20)
    hw.add_argument("--dir_blanking_us", type=int,   default=200)

    # ── Phase 1: Identification ───────────────────────────────────────────────
    id_g = p.add_argument_group("Phase 1 — Identification")
    id_g.add_argument("--id_dur",       type=float, default=10.0)
    id_g.add_argument("--id_fs",        type=float, default=800.0)
    id_g.add_argument("--min_freq",     type=float, default=1.0)
    id_g.add_argument("--max_freq",     type=float, default=50.0)
    id_g.add_argument("--n_tones",      type=int,   default=3)
    id_g.add_argument("--skip_id",      action="store_true")
    id_g.add_argument("--manual_freqs", type=float, nargs="+", default=[],
                      metavar="HZ")

    # ── Phase 2: LMS control ──────────────────────────────────────────────────
    lms = p.add_argument_group("Phase 2 — LMS Adaptive Control")
    lms.add_argument("--mu",               type=float, default=5e-4,
                     help="Amplitude/phase learning rate. "
                          "Rule of thumb: µ < 1/(fs·2n·signal²). "
                          "Typical range 1e-4 – 5e-3.")
    lms.add_argument("--mu_omega",         type=float, default=None,
                     help="Frequency learning rate. "
                          "Default: µ × 0.01.  Set 0 to disable frequency tracking.")
    lms.add_argument("--control_fs",       type=float, default=800.0)
    lms.add_argument("--max_amp_per_tone", type=float, default=20.0,
                     help="Requested max amplitude per tone (mm). Each tone's "
                          "actual limit is min(this, sps-derived limit for that tone).")
    lms.add_argument("--max_total_amp",    type=float, default=60.0)
    lms.add_argument("--init_amp_gain",    type=float, default=0.0)
    lms.add_argument("--cancel_dur",       type=float, default=0.0)

    # ── FFT frequency tracker ─────────────────────────────────────────────────
    fft = p.add_argument_group("FFT Frequency Tracker")
    fft.add_argument("--fft_tracker",     action="store_true",
                     help="Enable the parallel FFT-based frequency tracker. "
                          "When ON, the gradient ω update in the LMS thread is "
                          "automatically disabled (single-writer invariant).")
    fft.add_argument("--fft_window_sec",  type=float, default=2.0,
                     help="FFT window length [s]. Larger = finer resolution, "
                          "slower response. 2.0s gives ~0.5 Hz bin width with "
                          "parabolic interpolation reaching ~0.01-0.05 Hz.")
    fft.add_argument("--fft_update_sec",  type=float, default=1.0,
                     help="How often the FFT thread recomputes (s). Should be "
                          "≥ fft_window_sec/2 to avoid correlated estimates.")
    fft.add_argument("--fft_max_step_hz", type=float, default=0.05,
                     help="Maximum frequency change per FFT update [Hz]. "
                          "Limits per-update phase jumps in compute_chunk "
                          "(see KNOWN ISSUE in module docstring).")
    fft.add_argument("--fft_band_pad_hz", type=float, default=1.0,
                     help="The FFT search band is [f_initial - pad, "
                          "f_initial + pad] for each tone. Keeps tracker from "
                          "wandering into resonances or harmonics.")

    # ── Simulation ────────────────────────────────────────────────────────────
    sim = p.add_argument_group("Simulation")
    sim.add_argument("--simulate",        action="store_true")
    sim.add_argument("--sim_plant_gain",  type=float, default=DEFAULT_SIM_PLANT_GAIN)

    # ── Logging & plotting ────────────────────────────────────────────────────
    log = p.add_argument_group("Logging & Plotting")
    log.add_argument("--log_hz",   type=float, default=100.0,
                     help="Rate at which LMS state is logged [Hz]. Default 100.")
    log.add_argument("--no_plot",  action="store_true",
                     help="Skip plot generation after cancellation.")
    log.add_argument("--plot_out", type=str,   default="vss_run_plot.png",
                     help="Output filename for the convergence plot.")

    # ── Misc ──────────────────────────────────────────────────────────────────
    p.add_argument("--status_hz",        type=float, default=2.0)
    p.add_argument("--no_travel_safety", action="store_true")

    return p.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
# Phase 1 — Identification
# ══════════════════════════════════════════════════════════════════════════════

def identify_disturbance(args) -> list:
    """
    Collect IMU time series, compute single-sided FFT, pick top dominant
    peaks in [min_freq, max_freq].

    Returns list of (freq_hz, accel_amp) tuples, length ≤ args.n_tones.
    """
    print(f"\n{'─'*56}")
    print(f"  PHASE 1 — IDENTIFY")
    print(f"  Sampling {args.id_dur:.1f} s  at  {args.id_fs:.0f} Hz ...")
    print(f"  Band: [{args.min_freq:.1f}, {args.max_freq:.1f}] Hz   "
          f"top {args.n_tones} tone(s)")
    print(f"{'─'*56}")

    n_target = int(round(args.id_dur * args.id_fs))
    samples  = np.zeros(n_target)
    t_stamps = np.zeros(n_target)
    period   = 1.0 / args.id_fs

    t0     = time.perf_counter()
    next_t = t0

    for i in range(n_target):
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)
            now = time.perf_counter()
        t = now - t0

        if args.simulate:
            samples[i] = sim_disturbance(t)
        else:
            samples[i] = hw_read_accel()

        t_stamps[i] = now
        next_t += period

        if i % max(1, int(args.id_fs)) == 0:
            pct = 100 * i / n_target
            print(f"  {pct:3.0f}%  t={t:.1f}s  last={samples[i]:+.4f} m/s²")

    dts     = np.diff(t_stamps)
    fs_meas = 1.0 / np.mean(dts) if len(dts) else args.id_fs
    print(f"  Measured fs: {fs_meas:.1f} Hz")

    x   = samples - np.mean(samples)
    n   = len(x)
    win = np.hanning(n)
    cg  = np.mean(win)
    X   = np.fft.rfft(x * win)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs_meas)
    amps  = np.abs(X) / (n * cg)
    if n > 1:
        amps[1:-1] *= 2.0

    if len(freqs) < 2:
        return []
    bin_hz     = freqs[1] - freqs[0]
    guard_bins = max(2, int(2.0 / bin_hz))

    mask = (freqs >= args.min_freq) & (freqs <= args.max_freq)
    idxs = np.where(mask)[0]

    candidates = [
        i for i in idxs[1:-1]
        if amps[i] > amps[i - 1] and amps[i] >= amps[i + 1]
    ]
    candidates.sort(key=lambda i: amps[i], reverse=True)

    chosen     = []
    suppressed = np.zeros(len(amps), dtype=bool)
    for idx in candidates:
        if suppressed[idx]:
            continue
        chosen.append(idx)
        lo = max(0, idx - guard_bins)
        hi = min(len(amps), idx + guard_bins + 1)
        suppressed[lo:hi] = True
        if len(chosen) >= args.n_tones:
            break

    peaks = [(float(freqs[i]), float(amps[i])) for i in chosen]

    print(f"\n  Identified {len(peaks)} dominant tone(s):")
    for j, (f, a) in enumerate(peaks, 1):
        disp_mm = a / (2 * math.pi * f) ** 2 * 1000
        print(f"    {j})  {f:8.3f} Hz  |  {a:.4g} m/s²  "
              f"|  ~{disp_mm:.4f} mm structural disp.")

    return peaks

# ══════════════════════════════════════════════════════════════════════════════
# LMS coefficient helpers
# ══════════════════════════════════════════════════════════════════════════════

def clamp_coeffs(coeffs: list, per_tone_max_amp, max_total: float):
    """
    In-place amplitude clamp on [[C₀,S₀], ...] list.

    per_tone_max_amp may be a scalar (same limit for all tones) or a list
    with one entry per tone.  Using a list allows each tone to have its own
    sps-derived limit without penalising low-frequency tones with the tighter
    constraint that applies to high-frequency tones.
    """
    amps = []
    for i, cs in enumerate(coeffs):
        limit = (per_tone_max_amp[i]
                 if isinstance(per_tone_max_amp, (list, tuple))
                 else per_tone_max_amp)
        A = math.sqrt(cs[0] ** 2 + cs[1] ** 2)
        if limit > 0 and A > limit:
            r = limit / A
            cs[0] *= r
            cs[1] *= r
            A = limit
        amps.append(A)

    total = sum(amps)
    if total > max_total > 0:
        r = max_total / total
        for cs in coeffs:
            cs[0] *= r
            cs[1] *= r


def coeffs_to_aphi(coeffs: list):
    """[[C, S], ...] → (A_list, PHI_list) for compute_chunk."""
    A   = [math.sqrt(cs[0] ** 2 + cs[1] ** 2) for cs in coeffs]
    PHI = [math.atan2(cs[0], cs[1])            for cs in coeffs]
    return A, PHI

# ══════════════════════════════════════════════════════════════════════════════
# Convergence plot
# ══════════════════════════════════════════════════════════════════════════════

def plot_run(log_t, log_e, log_C, log_S, log_f, args):
    """
    Generate 4-panel convergence plot from logged LMS data.
    Mirrors the panels in vss_lms_sim.py for direct comparison.
    """
    n = len(log_t)
    if n < 2:
        print("  Not enough data to plot.")
        return

    log_t = np.array(log_t)
    log_e = np.array(log_e)
    log_C = np.array(log_C)
    log_S = np.array(log_S)
    log_f = np.array(log_f)
    log_A = np.sqrt(log_C**2 + log_S**2)

    # Rolling e_rms — 2-second window at log_hz
    window = max(1, int(2.0 * args.log_hz))
    e_rms = np.array([
        np.sqrt(np.mean(log_e[max(0, i - window):i + 1]**2))
        for i in range(n)
    ])

    mu_omega = args.mu_omega if args.mu_omega is not None else args.mu * 0.01

    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    fig.suptitle(
        f"VSS Hardware Run\n"
        f"µ={args.mu}  µω={mu_omega}  fs={args.control_fs:.0f}Hz  "
        f"dur={log_t[-1]:.1f}s",
        fontsize=10
    )

    # Panel 1 — residual error
    ax = axes[0]
    ax.plot(log_t, log_e, alpha=0.25, color='gray', linewidth=0.4, label='e(t)')
    ax.plot(log_t, e_rms, color='crimson', linewidth=1.5, label='e_rms (2s window)')
    ax.axhline(0, color='black', linewidth=0.5)
    ax.set_ylabel('Error [m/s²]')
    ax.set_title('Residual Error  (should fall monotonically if converging)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 2 — actuator amplitude
    ax = axes[1]
    ax.plot(log_t, log_A, color='steelblue', linewidth=1.5)
    ax.axhline(args.max_amp_per_tone, color='red', linestyle='--',
               linewidth=0.8, label=f'requested max = {args.max_amp_per_tone}mm')
    ax.set_ylabel('Amplitude [mm]')
    ax.set_title('Actuator Amplitude |C + jS|  (tone 1 — should grow then plateau)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3 — C and S coefficients
    ax = axes[2]
    ax.plot(log_t, log_C, linewidth=1.0, label='C  (cos coeff)', color='royalblue')
    ax.plot(log_t, log_S, linewidth=1.0, label='S  (sin coeff)', color='darkorange')
    ax.axhline(0, color='black', linewidth=0.5)
    ax.set_ylabel('[mm]')
    ax.set_title('LMS Coefficients  (should settle; continuous rotation = diverging)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 4 — tracked frequency
    ax = axes[3]
    ax.plot(log_t, log_f, color='seagreen', linewidth=1.5)
    if len(log_f):
        ax.axhline(log_f[0], color='black', linestyle='--',
                   linewidth=0.8, label=f'initial f = {log_f[0]:.3f} Hz')
    ax.set_ylabel('Frequency [Hz]')
    ax.set_xlabel('Time [s]')
    ax.set_title('Tracked Frequency')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.plot_out, dpi=150)
    print(f"  Plot saved: {args.plot_out}")
    plt.close(fig)

# ══════════════════════════════════════════════════════════════════════════════
# DMA waveform engine
# ══════════════════════════════════════════════════════════════════════════════

def compute_chunk(t_start, chunk_s, A, F, PHI, k,
                  max_sps, deadband_sps,
                  step_pin, dir_pin, last_dir, dir_invert,
                  dir_blanking_us, dir_setup_us,
                  step_phis):
    """
    Build one DMA waveform chunk covering [t_start, t_start + chunk_s).
    Returns (pulses, end_dir, t_end, net_steps).

    PHASE FRAME — fixes the absolute-time phase bug
    ────────────────────────────────────────────────
    Earlier versions evaluated  cos(2π·F·t + PHI)  using absolute time t.
    When F changes between chunks (FFT tracker, gradient tracker), the
    stepper's commanded phase jumps by  2π·ΔF·t  while the LMS's own phase
    accumulator continues smoothly.  Result: every frequency update
    re-randomizes the cancellation alignment.

    Fix: step_phis is a per-tone phase accumulator that evolves identically
    to the LMS phi accumulator (φ += ω·dt per substep).  It persists across
    chunk calls.  When F changes, step_phis continues from where it was —
    only the *rate* of evolution changes, never the value.

    step_phis is mutated IN PLACE.  Caller must initialise once (zeros) and
    pass the same list to every compute_chunk call.
    """
    STEP_BIT      = 1 << step_pin
    DIR_BIT       = 1 << dir_pin
    MIN_PERIOD_US = PULSE_HIGH_US + 1

    pulses      = []
    t           = t_start
    t_end       = t_start + chunk_s
    current_dir = last_dir
    net_steps   = 0

    n_tones = len(A)
    omegas  = [2.0 * math.pi * F[i] for i in range(n_tones)]

    def advance_phases(dt):
        for i in range(n_tones):
            step_phis[i] += omegas[i] * dt
            if step_phis[i] > math.pi:
                step_phis[i] -= 2.0 * math.pi

    while t < t_end:
        # NOTE: cos(step_phis[i] + PHI[i]), NOT cos(2π·F·t + PHI)
        v = sum(
            A[i] * omegas[i] * math.cos(step_phis[i] + PHI[i])
            for i in range(n_tones) if A[i] and F[i]
        )
        sps = min(abs(v) * k, max_sps)

        if sps < deadband_sps:
            wait_us = min(1000, max(1, int((t_end - t) * 1e6)))
            pulses.append(pigpio.pulse(0, 0, wait_us))
            dt = wait_us / 1e6
            advance_phases(dt)
            t += dt
            continue

        logical_dir = int(v >= 0.0) ^ int(dir_invert)
        if logical_dir != current_dir:
            if dir_blanking_us > 0:
                pulses.append(pigpio.pulse(0, 0, dir_blanking_us))
            pulses.append(pigpio.pulse(
                DIR_BIT if logical_dir else 0,
                0       if logical_dir else DIR_BIT,
                dir_setup_us,
            ))
            # NOTE: original code did NOT advance t for blanking/setup
            # (small pre-existing timing bug, ~220 µs per direction change).
            # Preserved here to avoid behavior change beyond the phase fix.
            current_dir = logical_dir

        period_us = max(MIN_PERIOD_US, int(round(1e6 / sps)))
        pulses.append(pigpio.pulse(STEP_BIT, 0, PULSE_HIGH_US))
        pulses.append(pigpio.pulse(0, STEP_BIT, period_us - PULSE_HIGH_US))

        phys_dir   = current_dir ^ int(dir_invert)
        net_steps += 1 if phys_dir else -1
        dt         = period_us / 1e6
        advance_phases(dt)
        t         += dt

    return pulses, current_dir, t_end, net_steps


def wave_dur_s(pulses) -> float:
    return sum(p.delay for p in pulses) / 1e6

# ══════════════════════════════════════════════════════════════════════════════
# FFT Frequency Tracker  —  parallel, robust to LMS instability
# ══════════════════════════════════════════════════════════════════════════════

class FFTFreqTracker(threading.Thread):
    """
    Parallel FFT-based frequency tracker.  Bypasses the gradient tracker's
    chicken-and-egg deadlock (gradient ω update depends on stable C, S, but
    C, S can't stabilize until ω is locked).

    Sole writer of `freqs_shared` while running.  The LMS thread auto-disables
    its own gradient ω update when this thread is active, preserving a
    single-writer invariant without locks on the freq state itself
    (single-float writes are atomic under the GIL).

    DESIGN NOTES
    ────────────
    • Window length controls bin resolution: Δf_bin = 1 / window_sec.
      Parabolic interpolation around the peak typically reaches ~Δf_bin/20
      for a clean single tone.

    • Update interval should be ≥ window_sec/2.  Faster updates produce
      correlated estimates (consecutive windows share most of their data)
      without adding information.

    • Per-update step is clamped to ±max_step_hz.  Originally introduced
      to limit phase jumps caused by the absolute-time phase bug in
      compute_chunk; that bug is now fixed (phase accumulator), so this
      clamp's main remaining purpose is robustness against transient FFT
      mis-locks (e.g., a noise spike briefly outranking the true peak).
      A value of 0.05–0.10 Hz is reasonable for typical drift rates.

    • Search band is [f_init - band_pad, f_init + band_pad] PER TONE.
      Prevents the tracker from latching onto a resonance peak or harmonic
      that's stronger than the disturbance fundamental.  band_pad should
      bound the worst-case drift you observed in motor_drift_char.py.
    """

    def __init__(self, imu_buffer, buffer_lock, freqs_shared, freqs_lock,
                 fs, window_sec, update_sec, max_step_hz, band_pad_hz,
                 init_freqs, running):
        super().__init__(daemon=True, name="fft-tracker")
        self.imu_buffer  = imu_buffer
        self.buffer_lock = buffer_lock
        self.freqs_shared = freqs_shared      # list[float], one per tone
        self.freqs_lock  = freqs_lock         # protects multi-element reads
        self.fs          = fs
        self.N           = int(window_sec * fs)
        self.update_sec  = update_sec
        self.max_step    = max_step_hz
        self.band_pad    = band_pad_hz
        self.init_freqs  = list(init_freqs)
        self.running     = running
        self.hann        = np.hanning(self.N)

        # Diagnostics — read by status thread for printing
        self.last_update_t  = 0.0
        self.last_f_meas    = list(init_freqs)   # raw FFT result (pre-clamp)
        self.update_count   = 0
        self.skip_count     = 0   # times we skipped (buffer not full enough)

    def _peak_in_band(self, spectrum, freqs_bin, f_lo, f_hi):
        """Find dominant peak in [f_lo, f_hi] with parabolic interpolation."""
        mask = (freqs_bin >= f_lo) & (freqs_bin <= f_hi)
        band_idx = np.where(mask)[0]
        if band_idx.size == 0:
            return None
        k_local = int(np.argmax(spectrum[band_idx]))
        k = band_idx[k_local]
        if 0 < k < len(spectrum) - 1:
            y0, y1, y2 = spectrum[k-1], spectrum[k], spectrum[k+1]
            denom = (y0 - 2.0 * y1 + y2)
            delta = 0.5 * (y0 - y2) / denom if denom != 0 else 0.0
            # Guard against runaway interpolation for non-parabolic peaks
            if abs(delta) > 1.0:
                delta = 0.0
        else:
            delta = 0.0
        return (k + delta) * self.fs / self.N

    def run(self):
        # Wait for buffer to fill before the first FFT
        while self.running[0]:
            with self.buffer_lock:
                ready = len(self.imu_buffer) >= self.N
            if ready:
                break
            time.sleep(0.1)

        freqs_bin = np.fft.rfftfreq(self.N, 1.0 / self.fs)
        next_t    = time.monotonic()

        while self.running[0]:
            now = time.monotonic()
            if now < next_t:
                time.sleep(min(0.05, next_t - now))
                continue
            next_t += self.update_sec

            # Snapshot the most recent N samples
            with self.buffer_lock:
                if len(self.imu_buffer) < self.N:
                    self.skip_count += 1
                    continue
                samples = np.fromiter(self.imu_buffer, dtype=np.float64,
                                      count=len(self.imu_buffer))[-self.N:]

            samples = samples - samples.mean()
            spec    = np.abs(np.fft.rfft(samples * self.hann))

            # Per-tone search in a tight band around init freq
            new_freqs = []
            for i, f_init in enumerate(self.init_freqs):
                f_lo = max(0.5, f_init - self.band_pad)
                f_hi = f_init + self.band_pad
                f_peak = self._peak_in_band(spec, freqs_bin, f_lo, f_hi)
                if f_peak is None:
                    new_freqs.append(self.freqs_shared[i])  # keep current
                    continue
                self.last_f_meas[i] = float(f_peak)
                # Clamp per-update step
                with self.freqs_lock:
                    f_cur = self.freqs_shared[i]
                step = f_peak - f_cur
                if step > self.max_step:
                    step = self.max_step
                elif step < -self.max_step:
                    step = -self.max_step
                new_freqs.append(f_cur + step)

            # Single bulk write under lock (multi-element atomicity)
            with self.freqs_lock:
                for i, f_new in enumerate(new_freqs):
                    self.freqs_shared[i] = f_new

            self.last_update_t = now
            self.update_count += 1


# ══════════════════════════════════════════════════════════════════════════════
# LMS thread  —  adapts C, S, and ω
# ══════════════════════════════════════════════════════════════════════════════

def lms_thread_fn(freqs, lms_coeffs, lms_freqs, lms_lock, t_start,
                  args, running, lms_stats, log_buf, per_tone_max_amp,
                  freqs_shared, freqs_lock, imu_buffer, buffer_lock,
                  fft_tracker_active):
    """
    Runs at ~control_fs Hz.  Adapts amplitude (C, S) and, if the gradient
    tracker is active (µ_omega > 0 AND fft_tracker_active is False),
    frequency (ω) for each tone.  Logs state to log_buf at --log_hz rate.

    per_tone_max_amp is a list of per-tone amplitude limits (mm), derived
    from --max_amp_per_tone and the per-tone sps constraint.

    KEY DESIGN: Phase accumulator
    ─────────────────────────────
    φᵢ += ωᵢ · dt each sample replaces ωᵢ·t so that when ω changes
    the phase evolves continuously with no discontinuity.

    KEY DESIGN: Single-writer freq invariant
    ────────────────────────────────────────
    Frequency state lives in freqs_shared.  Exactly one thread writes it:
      • If fft_tracker_active: the FFT thread is sole writer.
        The gradient ω update here is skipped.
      • Otherwise: this thread is sole writer (via the gradient update),
        and freqs_shared is updated each iteration.
    Either way, no write-write races on the freq state.

    Frequency gradient (gradient mode only):
      ∂x/∂ω ≈ Sᵢ·cos(φᵢ) − Cᵢ·sin(φᵢ)   (quadrature signal)
    """
    n        = len(freqs)
    phis     = [0.0] * n
    period   = 1.0 / args.control_fs
    dt       = period
    mu       = args.mu
    mu_omega = args.mu_omega if args.mu_omega is not None else mu * 0.01
    gradient_freq_active = (mu_omega > 0) and (not fft_tracker_active)

    log_every  = max(1, int(args.control_fs / args.log_hz))
    log_ticker = 0

    next_t = time.monotonic()

    while running[0]:
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
            now = time.monotonic()

        # ── Read the latest shared frequencies (FFT tracker may have updated) ─
        with freqs_lock:
            omegas = [2.0 * math.pi * f for f in freqs_shared]

        # ── Advance phase accumulators ────────────────────────────────────────
        for i in range(n):
            phis[i] += omegas[i] * dt
            if phis[i] > math.pi:
                phis[i] -= 2.0 * math.pi

        # ── Read error signal ─────────────────────────────────────────────────
        if args.simulate:
            with lms_lock:
                snap = [list(c) for c in lms_coeffs]
            t_sim = now - t_start
            e = sim_read_accel(t_sim, snap, freqs, args.sim_plant_gain)
        else:
            e = hw_read_accel()

        # ── Push to IMU ring buffer for the FFT tracker ──────────────────────
        # deque.append is thread-safe under the GIL; the lock is only needed
        # when we want a self-consistent snapshot of N elements (FFT thread).
        with buffer_lock:
            imu_buffer.append(e)

        # ── LMS gradient steps ────────────────────────────────────────────────
        with lms_lock:
            for i in range(n):
                cos_ref = math.cos(phis[i])
                sin_ref = math.sin(phis[i])
                C = lms_coeffs[i][0]
                S = lms_coeffs[i][1]

                lms_coeffs[i][0] -= mu * e * cos_ref
                lms_coeffs[i][1] -= mu * e * sin_ref

                if gradient_freq_active:
                    quad = S * cos_ref - C * sin_ref
                    omegas[i] -= mu_omega * e * quad
                    f_hz = omegas[i] / (2.0 * math.pi)
                    f_hz = max(args.min_freq, min(args.max_freq, f_hz))
                    omegas[i] = 2.0 * math.pi * f_hz

            # Each tone clamped to its own sps-derived limit
            clamp_coeffs(lms_coeffs, per_tone_max_amp, args.max_total_amp)

            for i in range(n):
                lms_freqs[i] = omegas[i] / (2.0 * math.pi)

        # ── Write back freqs_shared if we (the LMS) are the sole writer ──────
        if gradient_freq_active:
            with freqs_lock:
                for i in range(n):
                    freqs_shared[i] = omegas[i] / (2.0 * math.pi)

        lms_stats['e']     = e
        lms_stats['t']     = now - t_start
        lms_stats['freqs'] = [o / (2.0 * math.pi) for o in omegas]

        # ── Periodic logging ──────────────────────────────────────────────────
        log_ticker += 1
        if log_ticker >= log_every:
            log_ticker = 0
            with lms_lock:
                C0 = lms_coeffs[0][0]
                S0 = lms_coeffs[0][1]
                f0 = lms_freqs[0]
            log_buf['t'].append(now - t_start)
            log_buf['e'].append(e)
            log_buf['C'].append(C0)
            log_buf['S'].append(S0)
            log_buf['f'].append(f0)

        next_t += period

# ══════════════════════════════════════════════════════════════════════════════
# Status thread
# ══════════════════════════════════════════════════════════════════════════════

def status_thread_fn(pi, lms_coeffs, lms_lock, shared, shared_lock,
                     args, running, init_freqs, lms_stats, per_tone_max_amp):
    """Periodic status print + pigpiod keepalive."""
    if args.status_hz <= 0:
        return

    period  = 1.0 / args.status_hz
    next_p  = time.monotonic()
    next_ka = time.monotonic() + 2.0

    while running[0]:
        now = time.monotonic()

        if now >= next_ka:
            try:
                pi.get_current_tick()
            except Exception:
                pass
            next_ka = now + 2.0

        if now >= next_p:
            with lms_lock:
                snap = [list(c) for c in lms_coeffs]
            with shared_lock:
                sh = dict(shared)

            tracked_freqs = lms_stats.get('freqs', init_freqs)
            total_A = sum(math.sqrt(cs[0] ** 2 + cs[1] ** 2) for cs in snap)
            print(
                f"\nt={sh.get('t', 0):7.3f}s  "
                f"x_est={sh.get('x_est', 0):+7.2f}mm  "
                f"ΣA={total_A:.2f}mm  "
                f"e={lms_stats.get('e', 0):+.4f} m/s²"
            )
            for i, f_init in enumerate(init_freqs):
                C, S   = snap[i]
                A      = math.sqrt(C ** 2 + S ** 2)
                phi    = math.degrees(math.atan2(C, S))
                f_now  = tracked_freqs[i] if i < len(tracked_freqs) else f_init
                limit  = (per_tone_max_amp[i]
                          if isinstance(per_tone_max_amp, (list, tuple))
                          else per_tone_max_amp)
                drift_marker = "*" if abs(f_now - f_init) > 0.05 else " "
                print(f"  tone{i+1}  {f_now:.3f}Hz{drift_marker} "
                      f"A={A:.3f}mm (max {limit:.1f}mm)  φ={phi:+.1f}°  "
                      f"C={C:+.4f}  S={S:+.4f}")

            next_p += period

        time.sleep(0.01)

# ══════════════════════════════════════════════════════════════════════════════
# Phase 2 — Cancellation
# ══════════════════════════════════════════════════════════════════════════════

def run_cancellation(pi, tones, args, k, per_tone_max_amp):
    """
    Initialises LMS coefficients, starts LMS and status threads,
    then runs the DMA double-buffer stepper loop.

    per_tone_max_amp : list of float
        Per-tone amplitude limits (mm), one per tone.  Computed in main()
        as min(--max_amp_per_tone, sps-derived limit for that tone's frequency).
        Each tone is clamped independently so low-frequency tones are not
        penalised by the tighter constraint that applies to high-frequency tones.
    """
    print(f"\n{'─'*56}")
    print(f"  PHASE 2 — CANCEL")
    print(f"{'─'*56}")

    pi.wave_clear()

    F_init = [f for f, _ in tones]
    n      = len(F_init)

    mu_omega = args.mu_omega if args.mu_omega is not None else args.mu * 0.01
    freq_tracking = mu_omega > 0
    print(f"  Frequency tracking: "
          f"{'ON  (µ_omega=' + str(mu_omega) + ')' if freq_tracking else 'OFF'}")

    # ── Initialise LMS coefficients ───────────────────────────────────────────
    lms_coeffs = []
    for i, (f, accel_amp) in enumerate(tones):
        if args.init_amp_gain > 0 and accel_amp > 0:
            disp_mm = accel_amp / (2 * math.pi * f) ** 2 * 1000
            A_init  = min(disp_mm * args.init_amp_gain, per_tone_max_amp[i])
            lms_coeffs.append([A_init, 0.0])
        else:
            lms_coeffs.append([0.0, 0.0])

    lms_freqs = list(F_init)

    print(f"  Per-tone amplitude limits (mm): "
          f"{[f'{v:.2f}' for v in per_tone_max_amp]}")
    print(f"  Initial amplitudes:")
    for i, (f, cs) in enumerate(zip(F_init, lms_coeffs)):
        A_i = math.sqrt(cs[0] ** 2 + cs[1] ** 2)
        print(f"    tone{i+1}  {f:.3f}Hz  A₀={A_i:.3f}mm")

    lms_lock    = threading.Lock()
    running     = [True]
    lms_stats   = {'e': 0.0, 't': 0.0, 'freqs': list(F_init)}
    shared      = {'t': 0.0, 'x_est': 0.0}
    shared_lock = threading.Lock()
    chunk_s     = args.chunk_ms / 1000.0
    log_buf     = {'t': [], 'e': [], 'C': [], 'S': [], 'f': []}

    # ── Shared frequency state (single-writer invariant) ─────────────────────
    # Writers:
    #   • FFT tracker thread  if args.fft_tracker (sole writer)
    #   • LMS thread          if not args.fft_tracker AND mu_omega > 0
    # Reader: main loop at chunk boundary; LMS thread at each iteration
    freqs_shared = list(F_init)
    freqs_lock   = threading.Lock()

    # ── IMU ring buffer for the FFT tracker ──────────────────────────────────
    # Sized for fft_window_sec at control_fs, plus a small margin for jitter.
    imu_buf_len  = int(args.fft_window_sec * args.control_fs * 1.5) + 100
    imu_buffer   = deque(maxlen=imu_buf_len)
    buffer_lock  = threading.Lock()

    print(f"  FFT tracker: {'ON' if args.fft_tracker else 'OFF'}")
    if args.fft_tracker:
        print(f"    window={args.fft_window_sec}s  update={args.fft_update_sec}s  "
              f"max_step={args.fft_max_step_hz}Hz  band_pad=±{args.fft_band_pad_hz}Hz")
        if mu_omega > 0:
            print(f"    Note: gradient ω update auto-disabled (FFT is sole writer)")

    # ── GPIO setup ────────────────────────────────────────────────────────────
    pi.set_mode(args.dir,   pigpio.OUTPUT)
    pi.set_mode(args.step,  pigpio.OUTPUT)
    pi.set_mode(ENABLE_PIN, pigpio.OUTPUT)
    pi.write(ENABLE_PIN, 1)   # enable backstop interlock
    pi.wave_clear()

    pi.write(args.dir, 1)
    time.sleep(args.dir_setup_us / 1e6)

    t_wall_start = time.monotonic()

    # ── Spawn threads ─────────────────────────────────────────────────────────
    lms_t = threading.Thread(
        target=lms_thread_fn,
        args=(F_init, lms_coeffs, lms_freqs, lms_lock,
              t_wall_start, args, running, lms_stats, log_buf,
              per_tone_max_amp,
              freqs_shared, freqs_lock, imu_buffer, buffer_lock,
              args.fft_tracker),
        daemon=True,
        name="lms-imu",
    )
    lms_t.start()

    fft_t = None
    if args.fft_tracker:
        fft_t = FFTFreqTracker(
            imu_buffer    = imu_buffer,
            buffer_lock   = buffer_lock,
            freqs_shared  = freqs_shared,
            freqs_lock    = freqs_lock,
            fs            = args.control_fs,
            window_sec    = args.fft_window_sec,
            update_sec    = args.fft_update_sec,
            max_step_hz   = args.fft_max_step_hz,
            band_pad_hz   = args.fft_band_pad_hz,
            init_freqs    = F_init,
            running       = running,
        )
        fft_t.start()

    stat_t = threading.Thread(
        target=status_thread_fn,
        args=(pi, lms_coeffs, lms_lock, shared, shared_lock,
              args, running, F_init, lms_stats, per_tone_max_amp),
        daemon=True,
        name="status",
    )
    stat_t.start()

    t_phase    = 0.0
    s_est      = 0.0
    last_dir   = 1
    t_end_wall = (t_wall_start + args.cancel_dur) if args.cancel_dur > 0 else None

    # Per-tone stepper phase accumulators — see compute_chunk docstring.
    # Mutated in place by compute_chunk; persists across all chunk calls.
    step_phis  = [0.0] * n

    est_peak_sps = sum(
        math.sqrt(cs[0]**2 + cs[1]**2) * 2*math.pi*f * k
        for cs, f in zip(lms_coeffs, F_init)
    )
    print(f"\n  µ={args.mu}  µ_omega={mu_omega}  "
          f"control_fs={args.control_fs:.0f}Hz  chunk={chunk_s*1000:.0f}ms")
    print(f"  max_total_amp={args.max_total_amp}mm")
    print(f"  Tones: {[f'{f:.3f}Hz' for f in F_init]}")
    print(f"  Est. initial peak sps: {est_peak_sps:.0f}")
    print(f"  Logging at {args.log_hz:.0f} Hz → plot_out={args.plot_out}")
    print("  Ctrl-C to stop.\n")

    # ── Compute and fire first chunk ──────────────────────────────────────────
    with lms_lock:
        A, PHI = coeffs_to_aphi(lms_coeffs)
    with freqs_lock:
        F_cur = list(freqs_shared)

    pulses_c, last_dir, t_phase, steps_c = compute_chunk(
        t_phase, chunk_s, A, F_cur, PHI, k,
        args.max_sps, args.deadband_sps,
        args.step, args.dir, last_dir, args.dir_invert,
        args.dir_blanking_us, args.dir_setup_us,
        step_phis,
    )
    dur_c = wave_dur_s(pulses_c) or chunk_s

    pi.wave_add_generic(pulses_c)
    wave_c = pi.wave_create()
    pi.wave_send_once(wave_c)
    t_chunk_wall = time.monotonic()

    # ── Double-buffer main loop ───────────────────────────────────────────────
    #
    #   While chunk N plays via DMA, Python computes chunk N+1.
    #   Once N finishes: delete N, add N+1, send N+1 immediately.
    #   Only one waveform occupies the CB pool at a time.
    #
    try:
        while True:
            if t_end_wall and time.monotonic() >= t_end_wall:
                break

            with lms_lock:
                A, PHI = coeffs_to_aphi(lms_coeffs)
            with freqs_lock:
                F_cur  = list(freqs_shared)

            pulses_n, dir_n, t_next, steps_n = compute_chunk(
                t_phase, chunk_s, A, F_cur, PHI, k,
                args.max_sps, args.deadband_sps,
                args.step, args.dir, last_dir, args.dir_invert,
                args.dir_blanking_us, args.dir_setup_us,
                step_phis,
            )
            dur_n = wave_dur_s(pulses_n) or chunk_s

            sleep_s = dur_c - (time.monotonic() - t_chunk_wall) - 0.010
            if sleep_s > 0:
                time.sleep(sleep_s)
            while pi.wave_tx_busy():
                time.sleep(0.0001)

            s_est += steps_c
            x_est  = s_est / k

            if (not args.no_travel_safety) and abs(x_est) > HALF_TRAVEL_MM + 1.0:
                raise RuntimeError(
                    f"Travel safety trip: x_est={x_est:.2f}mm "
                    f"exceeds ±{HALF_TRAVEL_MM}mm"
                )

            # Delete current waveform before adding next — keeps CB pool at
            # single-waveform occupancy, preventing CB exhaustion at high sps.
            pi.wave_delete(wave_c)
            pi.wave_add_generic(pulses_n)
            wave_n = pi.wave_create()
            pi.wave_send_once(wave_n)
            t_chunk_wall = time.monotonic()

            if shared_lock.acquire(blocking=False):
                try:
                    shared['t']     = time.monotonic() - t_wall_start
                    shared['x_est'] = x_est
                finally:
                    shared_lock.release()

            wave_c   = wave_n
            dur_c    = dur_n
            steps_c  = steps_n
            last_dir = dir_n
            t_phase  = t_next

    except KeyboardInterrupt:
        pass

    finally:
        running[0] = False
        lms_t.join(timeout=0.5)
        stat_t.join(timeout=1.0)
        if fft_t is not None:
            fft_t.join(timeout=1.0)
        try:
            pi.wave_tx_stop()
            pi.wave_clear()
        except Exception:
            pass
        try:
            pi.write(args.step, 0)
            pi.write(args.dir,  0)
            pi.write(ENABLE_PIN, 0)
        except Exception:
            pass
        print("\nCancellation stopped.")
        print(f"Final x_est = {s_est / k:+.2f} mm  ({int(s_est):+d} steps)")

        with lms_lock:
            snap        = [list(c) for c in lms_coeffs]
            final_freqs = list(lms_freqs)
        print("\nFinal LMS coefficients:")
        for i, f_init in enumerate(F_init):
            C, S    = snap[i]
            A       = math.sqrt(C**2 + S**2)
            phi     = math.degrees(math.atan2(C, S))
            f_final = final_freqs[i]
            print(f"  tone{i+1}  {f_final:.3f}Hz (init {f_init:.3f}Hz)  "
                  f"A={A:.3f}mm  φ={phi:+.1f}°")

        if not args.no_plot:
            print("\nGenerating convergence plot...")
            plot_run(log_buf['t'], log_buf['e'], log_buf['C'],
                     log_buf['S'], log_buf['f'], args)

# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    args = parse_args()

    if not args.simulate and not _HW_IMU_AVAILABLE:
        sys.exit(
            "ERROR: adafruit_lsm6ds not found.\n"
            "Install:  pip3 install adafruit-circuitpython-lsm6ds\n"
            "Or run with --simulate for software-only testing."
        )

    if args.max_total_amp > HALF_TRAVEL_MM:
        sys.exit(f"ERROR: --max_total_amp ({args.max_total_amp}mm) exceeds "
                 f"HALF_TRAVEL_MM ({HALF_TRAVEL_MM}mm).")

    k = k_steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    mu_omega = args.mu_omega if args.mu_omega is not None else args.mu * 0.01

    print("═" * 56)
    print("  VSS CONTROLLER  —  Adaptive Vibration Suppression")
    print("═" * 56)
    print(f"  steps/mm   = {k:.4f}")
    print(f"  ±travel    = {HALF_TRAVEL_MM:.1f} mm  "
          f"(≈{HALF_TRAVEL_MM * k:.0f} steps)")
    print(f"  simulate   = {args.simulate}")
    print(f"  µ          = {args.mu}")
    print(f"  µ_omega    = {mu_omega}  "
          f"({'tracking ON' if mu_omega > 0 else 'tracking OFF'})")

    # ── Phase 1: Identify ─────────────────────────────────────────────────────
    if args.skip_id:
        if not args.manual_freqs:
            sys.exit("ERROR: --skip_id requires at least one --manual_freqs value.")
        if len(args.manual_freqs) > 3:
            sys.exit("ERROR: at most 3 tones supported.")
        tones = [(f, 0.0) for f in args.manual_freqs[:args.n_tones]]
        print(f"\n  Skipping identification.  Manual tones: {args.manual_freqs}")
    else:
        tones = identify_disturbance(args)
        if not tones:
            sys.exit(
                f"ERROR: no dominant tones found in "
                f"[{args.min_freq}, {args.max_freq}] Hz.\n"
                f"Try longer --id_dur, wider band, or --simulate."
            )

    # ── Compute per-tone amplitude limits ─────────────────────────────────────
    #
    #   Each tone's limit = min(--max_amp_per_tone, sps-safe limit for that f).
    #   Computed independently per tone so a high-frequency tone's tighter sps
    #   constraint does not reduce the limit for lower-frequency tones.
    #   The global args.max_amp_per_tone is never mutated.
    #
    per_tone_max_amp = []
    for f, _ in tones:
        sps_at_max = args.max_amp_per_tone * 2 * math.pi * f * k
        if sps_at_max > args.max_sps:
            safe_amp = args.max_sps / (2 * math.pi * f * k) * 0.95
            print(f"  WARNING: {f:.2f}Hz — amp limit for this tone set to "
                  f"{safe_amp:.2f}mm  (sps constraint; other tones unaffected)")
            per_tone_max_amp.append(safe_amp)
        else:
            per_tone_max_amp.append(args.max_amp_per_tone)

    # ── Phase 2: Cancel ───────────────────────────────────────────────────────
    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("ERROR: pigpiod not running.  "
                 "Start with:  sudo systemctl start pigpiod")

    try:
        run_cancellation(pi, tones, args, k, per_tone_max_amp)
    finally:
        pi.stop()

    print("Done.")


if __name__ == "__main__":
    main()

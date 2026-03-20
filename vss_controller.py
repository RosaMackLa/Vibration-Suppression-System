#!/usr/bin/env python3
"""
vss_controller.py  —  VSS closed-loop adaptive vibration suppression

PIPELINE
────────
  PHASE 1 — IDENTIFY : sample IMU for --id_dur seconds, FFT, pick top ≤N tones
  PHASE 2 — CANCEL   : DMA waveform stepper at identified frequencies;
                        LMS thread continuously adapts (C, S) coefficients
                        to minimise residual IMU acceleration

CONTROL LAW (per tone i, ωᵢ = 2πfᵢ)
──────────────────────────────────────
  Cancellation signal : x(t) = Σ [Cᵢ·cos(ωᵢt) + Sᵢ·sin(ωᵢt)]   [mm]
  Error               : e(t) = IMU x-axis acceleration              [m/s²]
  LMS update          : Cᵢ  -= µ · e(t) · cos(ωᵢt)
                        Sᵢ  -= µ · e(t) · sin(ωᵢt)

  Convergence: when the projections of e onto both basis functions reach zero
  the residual is orthogonal to each reference → LMS minimum MSE attained.
  The (C, S) basis jointly covers any amplitude and phase without explicit
  phase estimation; the loop self-corrects for plant phase shifts.

ARCHITECTURE
────────────
  Main thread   : DMA double-buffer loop (adapted from functionStepper_wave.py)
                  reads latest LMS coefficients at every chunk boundary
  LMS thread    : ~control_fs Hz — IMU read → gradient update → clamp
  Status thread : ~status_hz Hz print + pigpiod keepalive

SIMULATION MODE (--simulate)
──────────────────────────────
  A simple plant model replaces the hardware IMU:
    e(t) = disturbance(t) + plant_gain × Σ xᵢ(t)
  where plant_gain converts mm of counterweight travel to m/s² at the IMU.
  This lets you validate LMS convergence on a laptop with no hardware.

USAGE
─────
  # Real hardware (Pi 4B with DM556 + LSM6DSO):
  sudo chrt -f 50 python3 vss_controller.py --id_dur 10

  # Simulation (any machine):
  python3 vss_controller.py --simulate --id_dur 5 --mu 5e-3

  # Skip straight to known frequencies (bypass Phase 1):
  sudo chrt -f 50 python3 vss_controller.py --skip_id --manual_freqs 5.0 12.0

  # Faster convergence demo with seeded initial amplitude:
  sudo chrt -f 50 python3 vss_controller.py --id_dur 10 --init_amp_gain 0.3 --mu 2e-3
"""

import argparse
import math
import time
import threading
import sys
import random

import numpy as np
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

# Simulation plant model:
# A counterweight of this mass (kg) at this structure mass (kg) gives ~10 m/s²/m
# at the IMU, i.e. 0.01 m/s² per mm.  Tunable via --sim_plant_gain.
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

    Plant model (linear, no phase lag):
      effect = plant_gain × Σ xᵢ(t)
    where xᵢ(t) = Cᵢ·cos(ωᵢt) + Sᵢ·sin(ωᵢt)  [mm]

    At LMS convergence: effect → -disturbance → e → 0.
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
    hw.add_argument("--step",            type=int,   default=DEFAULT_STEP_BCM,
                    help="BCM STEP pin")
    hw.add_argument("--dir",             type=int,   default=DEFAULT_DIR_BCM,
                    help="BCM DIR pin")
    hw.add_argument("--dir_invert",      action="store_true",
                    help="Invert DIR polarity")
    hw.add_argument("--pulley_d_mm",     type=float, default=DEFAULT_PULLEY_D_MM,
                    help="Belt pulley diameter (mm)")
    hw.add_argument("--pulses_per_rev",  type=float, default=DEFAULT_PULSES_PER_REV,
                    help="Driver microstep setting (pulses/rev)")
    hw.add_argument("--max_sps",         type=float, default=8000.0,
                    help="Hard speed cap (steps/sec)")
    hw.add_argument("--deadband_sps",    type=float, default=20.0,
                    help="No-step zone near velocity zero-crossings (sps)")
    hw.add_argument("--chunk_ms",        type=float, default=50.0,
                    help="DMA waveform chunk duration (ms). "
                         "Also the max latency from stop command to motor halt.")
    hw.add_argument("--dir_setup_us",    type=int,   default=20,
                    help="DIR→STEP setup delay (µs)")
    hw.add_argument("--dir_blanking_us", type=int,   default=200,
                    help="STEP-off blanking before DIR flip (µs)")

    # ── Phase 1: Identification ───────────────────────────────────────────────
    id_g = p.add_argument_group("Phase 1 — Identification")
    id_g.add_argument("--id_dur",    type=float, default=10.0,
                      help="IMU capture duration for FFT identification (s)")
    id_g.add_argument("--id_fs",     type=float, default=800.0,
                      help="Target IMU sampling rate during identification (Hz)")
    id_g.add_argument("--min_freq",  type=float, default=1.0,
                      help="Lower bound of cancellation band (Hz)")
    id_g.add_argument("--max_freq",  type=float, default=50.0,
                      help="Upper bound of cancellation band (Hz)")
    id_g.add_argument("--n_tones",   type=int,   default=3,
                      help="Max number of dominant tones to cancel (1–3)")
    id_g.add_argument("--skip_id",   action="store_true",
                      help="Skip Phase 1 entirely; use --manual_freqs instead")
    id_g.add_argument("--manual_freqs", type=float, nargs="+", default=[],
                      metavar="HZ",
                      help="Tone frequencies to use when --skip_id is set")

    # ── Phase 2: LMS control ──────────────────────────────────────────────────
    lms = p.add_argument_group("Phase 2 — LMS Adaptive Control")
    lms.add_argument("--mu",               type=float, default=5e-4,
                     help="LMS learning rate µ.  Units: mm · s² / m "
                          "(i.e. how many mm the coefficient shifts per m/s² "
                          "of error per sample).  Typical range 1e-4 – 5e-3.")
    lms.add_argument("--control_fs",       type=float, default=800.0,
                     help="IMU sample rate during active cancellation (Hz)")
    lms.add_argument("--max_amp_per_tone", type=float, default=20.0,
                     help="Hard amplitude cap per tone (mm peak)")
    lms.add_argument("--max_total_amp",    type=float, default=60.0,
                     help="Hard cap on sum of all tone amplitudes (mm). "
                          "Must be ≤ HALF_TRAVEL_MM (75 mm).")
    lms.add_argument("--init_amp_gain",    type=float, default=0.0,
                     help="Scale factor applied to FFT-derived displacement "
                          "estimate to seed initial LMS amplitude. "
                          "0 = start from zero (safest). "
                          "0.1–0.5 gives faster early convergence.")
    lms.add_argument("--cancel_dur",       type=float, default=0.0,
                     help="Cancellation run time (s). 0 = run until Ctrl-C.")

    # ── Simulation ────────────────────────────────────────────────────────────
    sim = p.add_argument_group("Simulation")
    sim.add_argument("--simulate",        action="store_true",
                     help="Use synthetic IMU + simple plant model (no hardware)")
    sim.add_argument("--sim_plant_gain",  type=float, default=DEFAULT_SIM_PLANT_GAIN,
                     help="Simulated plant gain: m/s² per mm of counterweight "
                          "displacement.  Default models ~0.3 kg counterweight "
                          "on a ~30 kg structure.")

    # ── Misc ──────────────────────────────────────────────────────────────────
    p.add_argument("--status_hz",        type=float, default=2.0,
                   help="Status print rate (Hz). 0 = off.")
    p.add_argument("--no_travel_safety", action="store_true",
                   help="Disable software travel-limit check (hardware limits "
                        "via limit switches remain active).")

    return p.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
# Phase 1 — Identification
# ══════════════════════════════════════════════════════════════════════════════

def identify_disturbance(args) -> list:
    """
    Collect IMU time series, compute single-sided FFT, pick top dominant
    peaks in [min_freq, max_freq].

    Returns
    -------
    list of (freq_hz: float, accel_amp: float) tuples, sorted by amplitude,
    length ≤ args.n_tones.
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

    # Measured sample rate
    dts    = np.diff(t_stamps)
    fs_meas = 1.0 / np.mean(dts) if len(dts) else args.id_fs
    print(f"  Measured fs: {fs_meas:.1f} Hz")

    # ── FFT with Hann window ──────────────────────────────────────────────────
    x   = samples - np.mean(samples)          # remove DC
    n   = len(x)
    win = np.hanning(n)
    cg  = np.mean(win)                        # coherent gain correction
    X   = np.fft.rfft(x * win)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs_meas)
    amps  = np.abs(X) / (n * cg)
    if n > 1:
        amps[1:-1] *= 2.0                     # single-sided correction

    # ── Band-limited peak picking ─────────────────────────────────────────────
    if len(freqs) < 2:
        return []
    bin_hz     = freqs[1] - freqs[0]
    guard_bins = max(2, int(2.0 / bin_hz))    # 2 Hz guard band around each peak

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
        # Provide displacement context (structure, not VSS)
        disp_mm = a / (2 * math.pi * f) ** 2 * 1000
        print(f"    {j})  {f:8.3f} Hz  |  {a:.4g} m/s²  "
              f"|  ~{disp_mm:.4f} mm structural disp.")

    return peaks

# ══════════════════════════════════════════════════════════════════════════════
# LMS coefficient helpers
# ══════════════════════════════════════════════════════════════════════════════

def clamp_coeffs(coeffs: list, max_per_tone: float, max_total: float):
    """
    In-place amplitude clamp on [[C₀,S₀], [C₁,S₁], ...] list.

    1. Each tone:  Aᵢ = √(Cᵢ²+Sᵢ²)  clamped to max_per_tone.
    2. Total:      Σ Aᵢ               clamped to max_total (proportional scale).

    Called inside lms_lock, so no additional locking needed.
    """
    amps = []
    for cs in coeffs:
        A = math.sqrt(cs[0] ** 2 + cs[1] ** 2)
        if A > max_per_tone > 0:
            r = max_per_tone / A
            cs[0] *= r
            cs[1] *= r
            A = max_per_tone
        amps.append(A)

    total = sum(amps)
    if total > max_total > 0:
        r = max_total / total
        for cs in coeffs:
            cs[0] *= r
            cs[1] *= r


def coeffs_to_aphi(coeffs: list):
    """
    [[C, S], ...] → (A_list, PHI_list) for compute_chunk.

    Derivation:
      C·cos(ωt) + S·sin(ωt) = A·sin(ωt + φ)
      → A = √(C²+S²),   φ = atan2(C, S)
    """
    A   = [math.sqrt(cs[0] ** 2 + cs[1] ** 2) for cs in coeffs]
    PHI = [math.atan2(cs[0], cs[1])            for cs in coeffs]
    return A, PHI

# ══════════════════════════════════════════════════════════════════════════════
# DMA waveform engine (ported from functionStepper_wave.py)
# ══════════════════════════════════════════════════════════════════════════════

def compute_chunk(t_start, chunk_s, A, F, PHI, k,
                  max_sps, deadband_sps,
                  step_pin, dir_pin, last_dir, dir_invert,
                  dir_blanking_us, dir_setup_us):
    """
    Build one DMA waveform chunk covering [t_start, t_start + chunk_s).

    Each step's inter-pulse gap encodes instantaneous velocity; DIR changes
    embed blanking + setup delays directly in the pulse stream so the DMA
    engine handles their timing with zero CPU involvement.

    Returns (pulses, end_dir, t_end, net_steps).
    """
    STEP_BIT      = 1 << step_pin
    DIR_BIT       = 1 << dir_pin
    MIN_PERIOD_US = PULSE_HIGH_US + 1

    pulses      = []
    t           = t_start
    t_end       = t_start + chunk_s
    current_dir = last_dir
    net_steps   = 0

    while t < t_end:
        # Instantaneous velocity: v = dx/dt = Σ Aᵢ·ωᵢ·cos(ωᵢt + φᵢ)  [mm/s]
        v = sum(
            A[i] * (2.0 * math.pi * F[i]) * math.cos(2.0 * math.pi * F[i] * t + PHI[i])
            for i in range(len(A)) if A[i] and F[i]
        )
        sps = min(abs(v) * k, max_sps)

        # Near zero-crossing: emit no-op delay to preserve phase clock
        if sps < deadband_sps:
            wait_us = min(1000, max(1, int((t_end - t) * 1e6)))
            pulses.append(pigpio.pulse(0, 0, wait_us))
            t += wait_us / 1e6
            continue

        # Direction change → blank STEP, flip DIR, wait setup (all in DMA)
        logical_dir = int(v >= 0.0) ^ int(dir_invert)
        if logical_dir != current_dir:
            if dir_blanking_us > 0:
                pulses.append(pigpio.pulse(0, 0, dir_blanking_us))
            pulses.append(pigpio.pulse(
                DIR_BIT if logical_dir else 0,
                0       if logical_dir else DIR_BIT,
                dir_setup_us,
            ))
            current_dir = logical_dir

        period_us = max(MIN_PERIOD_US, int(round(1e6 / sps)))
        pulses.append(pigpio.pulse(STEP_BIT, 0, PULSE_HIGH_US))
        pulses.append(pigpio.pulse(0, STEP_BIT, period_us - PULSE_HIGH_US))

        phys_dir   = current_dir ^ int(dir_invert)
        net_steps += 1 if phys_dir else -1
        t         += period_us / 1e6

    return pulses, current_dir, t_end, net_steps


def wave_dur_s(pulses) -> float:
    """Exact wall-clock duration of a pigpio pulse list."""
    return sum(p.delay for p in pulses) / 1e6

# ══════════════════════════════════════════════════════════════════════════════
# LMS thread
# ══════════════════════════════════════════════════════════════════════════════

def lms_thread_fn(freqs, lms_coeffs, lms_lock, t_start,
                  args, running, lms_stats):
    """
    Runs continuously at ~control_fs Hz.

    Each iteration:
      1. Read IMU (or simulation).
      2. For each tone i:
           Cᵢ -= µ · e · cos(ωᵢt)
           Sᵢ -= µ · e · sin(ωᵢt)
      3. Clamp amplitudes.

    WHY this converges:
      The gradient of E[e²] w.r.t. Cᵢ is 2·E[e · ∂e/∂Cᵢ].
      Standard LMS approximates ∂e/∂Cᵢ ≈ cos(ωᵢt) (identity plant assumption).
      The orthogonal (C, S) pair jointly spans all phases, so convergence is
      guaranteed for any plant phase shift as long as µ is small enough.

    µ TUNING GUIDE:
      Too large  → oscillation or divergence (clamp engages repeatedly)
      Too small  → slow convergence (many seconds to reach steady-state)
      Rule of thumb: µ < 1 / (fs · P · max_expected_signal²)
        where P = number of coefficients = 2 × n_tones.
      For IMU signal ~0.5 m/s², fs=800, P=6:  µ < 1/(800·6·0.25) ≈ 8e-4.
      Default 5e-4 is conservative. Raise to 2e-3 for faster demo convergence.
    """
    omegas = [2.0 * math.pi * f for f in freqs]
    period = 1.0 / args.control_fs
    mu     = args.mu
    n      = len(freqs)

    next_t = time.monotonic()

    while running[0]:
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
            now = time.monotonic()

        t = now - t_start

        # ── Read error signal ─────────────────────────────────────────────────
        # Hardware: plain I²C read.
        # Simulation: read coefficients snapshot BEFORE update so the plant
        # model sees the same coefficients that are currently playing.
        if args.simulate:
            with lms_lock:
                snap = [list(c) for c in lms_coeffs]
            e = sim_read_accel(t, snap, freqs, args.sim_plant_gain)
        else:
            e = hw_read_accel()

        # ── LMS gradient step ─────────────────────────────────────────────────
        with lms_lock:
            for i in range(n):
                wt = omegas[i] * t
                lms_coeffs[i][0] -= mu * e * math.cos(wt)    # C update
                lms_coeffs[i][1] -= mu * e * math.sin(wt)    # S update
            clamp_coeffs(lms_coeffs, args.max_amp_per_tone, args.max_total_amp)

        lms_stats['e'] = e
        lms_stats['t'] = t
        next_t += period

# ══════════════════════════════════════════════════════════════════════════════
# Status thread
# ══════════════════════════════════════════════════════════════════════════════

def status_thread_fn(pi, lms_coeffs, lms_lock, shared, shared_lock,
                     args, running, freqs, lms_stats):
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

            total_A = sum(math.sqrt(cs[0] ** 2 + cs[1] ** 2) for cs in snap)
            print(
                f"\nt={sh.get('t', 0):7.3f}s  "
                f"x_est={sh.get('x_est', 0):+7.2f}mm  "
                f"ΣA={total_A:.2f}mm  "
                f"e={lms_stats.get('e', 0):+.4f} m/s²"
            )
            for i, f in enumerate(freqs):
                C, S = snap[i]
                A   = math.sqrt(C ** 2 + S ** 2)
                phi = math.degrees(math.atan2(C, S))
                print(f"  tone{i+1}  {f:.2f}Hz  "
                      f"A={A:.3f}mm  φ={phi:+.1f}°  "
                      f"C={C:+.4f}  S={S:+.4f}")

            next_p += period

        time.sleep(0.01)

# ══════════════════════════════════════════════════════════════════════════════
# Phase 2 — Cancellation
# ══════════════════════════════════════════════════════════════════════════════

def run_cancellation(pi, tones, args, k):
    """
    tones : [(freq_hz, accel_amp), ...]  — from identify_disturbance()
            or [(freq_hz, 0.0), ...]     — from --manual_freqs

    Initialises LMS coefficients, starts LMS and status threads,
    then runs the DMA double-buffer stepper loop.

    Coefficient handoff: at each chunk boundary (~chunk_ms), the main loop
    takes a snapshot of the latest LMS coefficients, converts (C, S) → (A, φ),
    and feeds them to compute_chunk.  The LMS thread runs asynchronously and
    may update 40 times between chunk boundaries; only the snapshot at handoff
    matters.  This is the correct tradeoff: sub-chunk coefficient changes
    would misalign the waveform's internal position tracking.
    """
    print(f"\n{'─'*56}")
    print(f"  PHASE 2 — CANCEL")
    print(f"{'─'*56}")

    F = [f for f, _ in tones]
    n = len(F)

    # ── Initialise LMS coefficients ───────────────────────────────────────────
    # Strategy: if init_amp_gain > 0, seed from FFT-derived displacement
    # estimate.  Start as pure cosine (C=A, S=0) so LMS has a finite gradient
    # from sample 0.  If gain=0, start from zero — safest, but slower ramp-up.
    lms_coeffs = []
    for f, accel_amp in tones:
        if args.init_amp_gain > 0 and accel_amp > 0:
            disp_mm = accel_amp / (2 * math.pi * f) ** 2 * 1000  # structural mm
            A_init  = min(disp_mm * args.init_amp_gain, args.max_amp_per_tone)
            lms_coeffs.append([A_init, 0.0])   # C=A, S=0 → x(t)=A·cos(ωt)
        else:
            lms_coeffs.append([0.0, 0.0])

    print(f"  Initial amplitudes:")
    for i, (f, cs) in enumerate(zip(F, lms_coeffs)):
        A_i = math.sqrt(cs[0] ** 2 + cs[1] ** 2)
        print(f"    tone{i+1}  {f:.2f}Hz  A₀={A_i:.3f}mm")

    lms_lock    = threading.Lock()
    running     = [True]
    lms_stats   = {'e': 0.0, 't': 0.0}
    shared      = {'t': 0.0, 'x_est': 0.0}
    shared_lock = threading.Lock()
    chunk_s     = args.chunk_ms / 1000.0

    # ── GPIO setup ────────────────────────────────────────────────────────────
    pi.set_mode(args.dir,   pigpio.OUTPUT)
    pi.set_mode(args.step,  pigpio.OUTPUT)
    pi.set_mode(ENABLE_PIN, pigpio.OUTPUT)
    pi.write(ENABLE_PIN, 1)
    pi.wave_clear()

    # Set initial DIR before first pulse
    pi.write(args.dir, 1)
    time.sleep(args.dir_setup_us / 1e6)

    t_wall_start = time.monotonic()

    # ── Spawn threads ─────────────────────────────────────────────────────────
    lms_t = threading.Thread(
        target=lms_thread_fn,
        args=(F, lms_coeffs, lms_lock, t_wall_start, args, running, lms_stats),
        daemon=True,
        name="lms-imu",
    )
    lms_t.start()

    stat_t = threading.Thread(
        target=status_thread_fn,
        args=(pi, lms_coeffs, lms_lock, shared, shared_lock,
              args, running, F, lms_stats),
        daemon=True,
        name="status",
    )
    stat_t.start()

    t_phase    = 0.0
    s_est      = 0.0          # cumulative step count (signed)
    last_dir   = 1
    t_end_wall = (t_wall_start + args.cancel_dur) if args.cancel_dur > 0 else None

    # Print run parameters
    est_peak_sps = sum(
        math.sqrt(cs[0]**2 + cs[1]**2) * 2*math.pi*f * k
        for cs, f in zip(lms_coeffs, F)
    )
    print(f"\n  µ={args.mu}  control_fs={args.control_fs:.0f}Hz  "
          f"chunk={chunk_s*1000:.0f}ms")
    print(f"  max_amp_per_tone={args.max_amp_per_tone}mm  "
          f"max_total_amp={args.max_total_amp}mm")
    print(f"  Tones: {[f'{f:.2f}Hz' for f in F]}")
    print(f"  Est. initial peak sps: {est_peak_sps:.0f}")
    print("  Ctrl-C to stop.\n")

    # ── Compute and fire first chunk ──────────────────────────────────────────
    with lms_lock:
        A, PHI = coeffs_to_aphi(lms_coeffs)

    pulses_c, last_dir, t_phase, steps_c = compute_chunk(
        t_phase, chunk_s, A, F, PHI, k,
        args.max_sps, args.deadband_sps,
        args.step, args.dir, last_dir, args.dir_invert,
        args.dir_blanking_us, args.dir_setup_us,
    )
    dur_c = wave_dur_s(pulses_c) or chunk_s

    pi.wave_add_generic(pulses_c)
    wave_c = pi.wave_create()
    pi.wave_send_once(wave_c)
    t_chunk_wall = time.monotonic()

    # ── Double-buffer main loop ───────────────────────────────────────────────
    #
    #  While chunk N plays on DMA hardware, Python:
    #    1. Reads latest LMS coefficients (takes ~1 µs)
    #    2. Calls compute_chunk to build chunk N+1 (CPU-bound, ~5–15 ms)
    #    3. Sleeps until ~10 ms before chunk N finishes
    #    4. Polls wave_tx_busy() until done
    #    5. Fires chunk N+1 immediately (< 100 µs gap)
    #    6. Updates position integrator with exact step count from chunk N
    #
    try:
        while True:
            if t_end_wall and time.monotonic() >= t_end_wall:
                break

            # Snapshot LMS coefficients for next chunk
            with lms_lock:
                A, PHI = coeffs_to_aphi(lms_coeffs)

            # Build next chunk (runs while current chunk plays)
            pulses_n, dir_n, t_next, steps_n = compute_chunk(
                t_phase, chunk_s, A, F, PHI, k,
                args.max_sps, args.deadband_sps,
                args.step, args.dir, last_dir, args.dir_invert,
                args.dir_blanking_us, args.dir_setup_us,
            )
            dur_n = wave_dur_s(pulses_n) or chunk_s

            # Sleep until 10 ms before end of current chunk, then poll
            sleep_s = dur_c - (time.monotonic() - t_chunk_wall) - 0.010
            if sleep_s > 0:
                time.sleep(sleep_s)
            while pi.wave_tx_busy():
                time.sleep(0.0001)

            # ── Current chunk just finished ────────────────────────────────
            s_est += steps_c
            x_est  = s_est / k

            if (not args.no_travel_safety) and abs(x_est) > HALF_TRAVEL_MM + 1.0:
                raise RuntimeError(
                    f"Travel safety trip: x_est={x_est:.2f}mm "
                    f"exceeds ±{HALF_TRAVEL_MM}mm"
                )

            # Fire next chunk immediately
            pi.wave_add_generic(pulses_n)
            wave_n = pi.wave_create()
            pi.wave_send_once(wave_n)
            t_chunk_wall = time.monotonic()

            # Free the chunk that just finished
            pi.wave_delete(wave_c)

            # Update shared status (non-blocking skip if contended)
            if shared_lock.acquire(blocking=False):
                try:
                    shared['t']     = time.monotonic() - t_wall_start
                    shared['x_est'] = x_est
                finally:
                    shared_lock.release()

            # Rotate buffers
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
        print(f"Final x_est = {s_est / k:+.2f} mm  ({s_est:+d} steps)")

        # Print final LMS state
        with lms_lock:
            snap = [list(c) for c in lms_coeffs]
        print("\nFinal LMS coefficients:")
        for i, f in enumerate(F):
            C, S = snap[i]
            A   = math.sqrt(C**2 + S**2)
            phi = math.degrees(math.atan2(C, S))
            print(f"  tone{i+1}  {f:.2f}Hz  A={A:.3f}mm  φ={phi:+.1f}°")

# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    args = parse_args()

    # Validate
    if not args.simulate and not _HW_IMU_AVAILABLE:
        sys.exit(
            "ERROR: adafruit_lsm6ds not found.\n"
            "Install:  pip3 install adafruit-circuitpython-lsm6ds\n"
            "Or run with --simulate for software-only testing."
        )

    if args.max_total_amp > HALF_TRAVEL_MM:
        sys.exit(f"ERROR: --max_total_amp ({args.max_total_amp}mm) exceeds "
                 f"HALF_TRAVEL_MM ({HALF_TRAVEL_MM}mm).  Position integrator "
                 f"would trip the safety limit at full amplitude.")

    k = k_steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    print("═" * 56)
    print("  VSS CONTROLLER  —  Adaptive Vibration Suppression")
    print("═" * 56)
    print(f"  steps/mm   = {k:.4f}")
    print(f"  ±travel    = {HALF_TRAVEL_MM:.1f} mm  "
          f"(≈{HALF_TRAVEL_MM * k:.0f} steps)")
    print(f"  simulate   = {args.simulate}")
    print(f"  µ          = {args.mu}")

    # ── Phase 1: Identify ────────────────────────────────────────────────────
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

    # ── Validate tones against hardware limits ───────────────────────────────
    for f, _ in tones:
        peak_sps = args.max_amp_per_tone * 2 * math.pi * f * k
        if peak_sps > args.max_sps:
            safe_amp = args.max_sps / (2 * math.pi * f * k) * 0.95
            print(f"  WARNING: {f:.2f}Hz — max_amp_per_tone reduced to "
                  f"{safe_amp:.1f}mm (max_sps={args.max_sps:.0f} limit)")
            args.max_amp_per_tone = min(args.max_amp_per_tone, safe_amp)

    # ── Phase 2: Cancel ──────────────────────────────────────────────────────
    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("ERROR: pigpiod not running.  "
                 "Start with:  sudo systemctl start pigpiod")

    try:
        run_cancellation(pi, tones, args, k)
    finally:
        pi.stop()

    print("Done.")


if __name__ == "__main__":
    main()

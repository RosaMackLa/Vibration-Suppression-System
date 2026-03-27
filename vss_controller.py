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
  LMS thread    : ~control_fs Hz — IMU read → gradient update → clamp
  Status thread : ~status_hz Hz print (shows tracked frequency) + keepalive

USAGE
─────
  # Real hardware (Pi 4B with DM556 + LSM6DSO):
  sudo chrt -f 50 python3 vss_controller.py --id_dur 30 --dir_invert --max_sps 16000

  # Simulation (any machine):
  python3 vss_controller.py --simulate --id_dur 5 --mu 5e-3

  # Skip straight to known frequencies (bypass Phase 1):
  sudo chrt -f 50 python3 vss_controller.py --skip_id --manual_freqs 12.0

  # Adaptive frequency tracking enabled:
  sudo chrt -f 50 python3 vss_controller.py --id_dur 30 --dir_invert \\
      --max_sps 16000 --mu 5e-4 --mu_omega 5e-6
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
                     help="Amplitude/phase learning rate (mm·s²/m). "
                          "Rule of thumb: µ < 1/(fs·2n·signal²). "
                          "Typical range 1e-4 – 5e-3.")
    lms.add_argument("--mu_omega",         type=float, default=None,
                     help="Frequency learning rate (rad/s per m/s² per sample). "
                          "Default: µ × 0.01.  Set 0 to disable frequency tracking.")
    lms.add_argument("--control_fs",       type=float, default=800.0)
    lms.add_argument("--max_amp_per_tone", type=float, default=20.0)
    lms.add_argument("--max_total_amp",    type=float, default=60.0)
    lms.add_argument("--init_amp_gain",    type=float, default=0.0)
    lms.add_argument("--cancel_dur",       type=float, default=0.0)

    # ── Simulation ────────────────────────────────────────────────────────────
    sim = p.add_argument_group("Simulation")
    sim.add_argument("--simulate",        action="store_true")
    sim.add_argument("--sim_plant_gain",  type=float, default=DEFAULT_SIM_PLANT_GAIN)

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

def clamp_coeffs(coeffs: list, max_per_tone: float, max_total: float):
    """In-place amplitude clamp on [[C₀,S₀], ...] list."""
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
    """[[C, S], ...] → (A_list, PHI_list) for compute_chunk."""
    A   = [math.sqrt(cs[0] ** 2 + cs[1] ** 2) for cs in coeffs]
    PHI = [math.atan2(cs[0], cs[1])            for cs in coeffs]
    return A, PHI

# ══════════════════════════════════════════════════════════════════════════════
# DMA waveform engine
# ══════════════════════════════════════════════════════════════════════════════

def compute_chunk(t_start, chunk_s, A, F, PHI, k,
                  max_sps, deadband_sps,
                  step_pin, dir_pin, last_dir, dir_invert,
                  dir_blanking_us, dir_setup_us):
    """
    Build one DMA waveform chunk covering [t_start, t_start + chunk_s).
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
        v = sum(
            A[i] * (2.0 * math.pi * F[i]) * math.cos(2.0 * math.pi * F[i] * t + PHI[i])
            for i in range(len(A)) if A[i] and F[i]
        )
        sps = min(abs(v) * k, max_sps)

        if sps < deadband_sps:
            wait_us = min(1000, max(1, int((t_end - t) * 1e6)))
            pulses.append(pigpio.pulse(0, 0, wait_us))
            t += wait_us / 1e6
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
            current_dir = logical_dir

        period_us = max(MIN_PERIOD_US, int(round(1e6 / sps)))
        pulses.append(pigpio.pulse(STEP_BIT, 0, PULSE_HIGH_US))
        pulses.append(pigpio.pulse(0, STEP_BIT, period_us - PULSE_HIGH_US))

        phys_dir   = current_dir ^ int(dir_invert)
        net_steps += 1 if phys_dir else -1
        t         += period_us / 1e6

    return pulses, current_dir, t_end, net_steps


def wave_dur_s(pulses) -> float:
    return sum(p.delay for p in pulses) / 1e6

# ══════════════════════════════════════════════════════════════════════════════
# LMS thread  —  adapts C, S, and ω
# ══════════════════════════════════════════════════════════════════════════════

def lms_thread_fn(freqs, lms_coeffs, lms_freqs, lms_lock, t_start,
                  args, running, lms_stats):
    """
    Runs at ~control_fs Hz.  Adapts amplitude (C, S) and, if µ_omega > 0,
    frequency (ω) for each tone.

    KEY CHANGE vs. previous version
    ────────────────────────────────
    Phase accumulator replaces ωᵢ·t:
      φᵢ += ωᵢ · dt   each sample
    This ensures phase is continuous when ω changes mid-run.  Using ωᵢ·t
    directly would create a phase jump whenever ω is updated.

    Frequency gradient:
      ∂x/∂ω ≈ Sᵢ·cos(φᵢ) − Cᵢ·sin(φᵢ)   (quadrature signal)
    This is the component of x(t) that is 90° ahead of the current output.
    It points in the direction of increasing ω, so subtracting µ_omega·e·quad
    drives ω toward the disturbance frequency.

    µ_omega tuning:
      Start at µ × 0.01.  Too large → ω oscillates.  Too small → slow tracking.
      Frequency drift in a DC motor is slow (seconds), so small µ_omega is fine.
    """
    n        = len(freqs)
    omegas   = [2.0 * math.pi * f for f in freqs]   # local, adapted each step
    phis     = [0.0] * n                              # phase accumulators
    period   = 1.0 / args.control_fs
    dt       = period
    mu       = args.mu
    mu_omega = args.mu_omega if args.mu_omega is not None else mu * 0.01

    next_t = time.monotonic()

    while running[0]:
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
            now = time.monotonic()

        # ── Advance phase accumulators ────────────────────────────────────────
        # Must happen BEFORE the IMU read so cos/sin refs are current.
        for i in range(n):
            phis[i] += omegas[i] * dt
            # Keep phi in [-π, π] to avoid floating-point drift over hours
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

        # ── LMS gradient steps ────────────────────────────────────────────────
        with lms_lock:
            for i in range(n):
                cos_ref = math.cos(phis[i])
                sin_ref = math.sin(phis[i])
                C = lms_coeffs[i][0]
                S = lms_coeffs[i][1]

                # Amplitude / phase updates (unchanged from original)
                lms_coeffs[i][0] -= mu * e * cos_ref
                lms_coeffs[i][1] -= mu * e * sin_ref

                # Frequency update (new)
                if mu_omega > 0:
                    quad = S * cos_ref - C * sin_ref   # quadrature signal
                    omegas[i] -= mu_omega * e * quad

                    # Clamp ω to [min_freq, max_freq] band
                    f_hz = omegas[i] / (2.0 * math.pi)
                    f_hz = max(args.min_freq, min(args.max_freq, f_hz))
                    omegas[i] = 2.0 * math.pi * f_hz

            clamp_coeffs(lms_coeffs, args.max_amp_per_tone, args.max_total_amp)

            # Publish adapted frequencies for main loop + status thread
            for i in range(n):
                lms_freqs[i] = omegas[i] / (2.0 * math.pi)

        lms_stats['e']     = e
        lms_stats['t']     = now - t_start
        lms_stats['freqs'] = [o / (2.0 * math.pi) for o in omegas]

        next_t += period

# ══════════════════════════════════════════════════════════════════════════════
# Status thread
# ══════════════════════════════════════════════════════════════════════════════

def status_thread_fn(pi, lms_coeffs, lms_lock, shared, shared_lock,
                     args, running, init_freqs, lms_stats):
    """Periodic status print + pigpiod keepalive.
    Shows tracked (adapted) frequency alongside amplitude and phase."""
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

            # Use tracked frequencies from lms_stats if available
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
                # Show tracked frequency; mark with * if it has drifted > 0.05 Hz
                drift_marker = "*" if abs(f_now - f_init) > 0.05 else " "
                print(f"  tone{i+1}  {f_now:.3f}Hz{drift_marker} "
                      f"A={A:.3f}mm  φ={phi:+.1f}°  "
                      f"C={C:+.4f}  S={S:+.4f}")

            next_p += period

        time.sleep(0.01)

# ══════════════════════════════════════════════════════════════════════════════
# Phase 2 — Cancellation
# ══════════════════════════════════════════════════════════════════════════════

def run_cancellation(pi, tones, args, k):
    """
    Initialises LMS coefficients, starts LMS and status threads,
    then runs the DMA double-buffer stepper loop.

    NEW: lms_freqs is a shared mutable list that the LMS thread updates
    as ω adapts.  The main loop reads lms_freqs at each chunk boundary
    and passes the adapted frequency to compute_chunk so the DMA waveform
    always matches the current LMS reference signal.
    """
    print(f"\n{'─'*56}")
    print(f"  PHASE 2 — CANCEL")
    print(f"{'─'*56}")

    pi.wave_clear()

    F_init = [f for f, _ in tones]   # identified frequencies (fixed reference)
    n      = len(F_init)

    mu_omega = args.mu_omega if args.mu_omega is not None else args.mu * 0.01
    freq_tracking = mu_omega > 0
    print(f"  Frequency tracking: {'ON  (µ_omega=' + str(mu_omega) + ')' if freq_tracking else 'OFF'}")

    # ── Initialise LMS coefficients ───────────────────────────────────────────
    lms_coeffs = []
    for f, accel_amp in tones:
        if args.init_amp_gain > 0 and accel_amp > 0:
            disp_mm = accel_amp / (2 * math.pi * f) ** 2 * 1000
            A_init  = min(disp_mm * args.init_amp_gain, args.max_amp_per_tone)
            lms_coeffs.append([A_init, 0.0])
        else:
            lms_coeffs.append([0.0, 0.0])

    # Shared mutable frequency list — LMS thread writes, main loop reads
    lms_freqs = list(F_init)

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

    # ── GPIO setup ────────────────────────────────────────────────────────────
    pi.set_mode(args.dir,   pigpio.OUTPUT)
    pi.set_mode(args.step,  pigpio.OUTPUT)
    pi.set_mode(ENABLE_PIN, pigpio.OUTPUT)
    pi.write(ENABLE_PIN, 1)
    pi.wave_clear()

    pi.write(args.dir, 1)
    time.sleep(args.dir_setup_us / 1e6)

    t_wall_start = time.monotonic()

    # ── Spawn threads ─────────────────────────────────────────────────────────
    lms_t = threading.Thread(
        target=lms_thread_fn,
        args=(F_init, lms_coeffs, lms_freqs, lms_lock,
              t_wall_start, args, running, lms_stats),
        daemon=True,
        name="lms-imu",
    )
    lms_t.start()

    stat_t = threading.Thread(
        target=status_thread_fn,
        args=(pi, lms_coeffs, lms_lock, shared, shared_lock,
              args, running, F_init, lms_stats),
        daemon=True,
        name="status",
    )
    stat_t.start()

    t_phase    = 0.0
    s_est      = 0.0
    last_dir   = 1
    t_end_wall = (t_wall_start + args.cancel_dur) if args.cancel_dur > 0 else None

    est_peak_sps = sum(
        math.sqrt(cs[0]**2 + cs[1]**2) * 2*math.pi*f * k
        for cs, f in zip(lms_coeffs, F_init)
    )
    print(f"\n  µ={args.mu}  µ_omega={mu_omega}  "
          f"control_fs={args.control_fs:.0f}Hz  chunk={chunk_s*1000:.0f}ms")
    print(f"  max_amp_per_tone={args.max_amp_per_tone}mm  "
          f"max_total_amp={args.max_total_amp}mm")
    print(f"  Tones: {[f'{f:.3f}Hz' for f in F_init]}")
    print(f"  Est. initial peak sps: {est_peak_sps:.0f}")
    print("  Ctrl-C to stop.\n")

    # ── Compute and fire first chunk ──────────────────────────────────────────
    with lms_lock:
        A, PHI = coeffs_to_aphi(lms_coeffs)
        F_cur  = list(lms_freqs)

    pulses_c, last_dir, t_phase, steps_c = compute_chunk(
        t_phase, chunk_s, A, F_cur, PHI, k,
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
    try:
        while True:
            if t_end_wall and time.monotonic() >= t_end_wall:
                break

            # Snapshot LMS state — coefficients AND adapted frequencies
            with lms_lock:
                A, PHI = coeffs_to_aphi(lms_coeffs)
                F_cur  = list(lms_freqs)   # ← adapted frequency for chunk

            pulses_n, dir_n, t_next, steps_n = compute_chunk(
                t_phase, chunk_s, A, F_cur, PHI, k,
                args.max_sps, args.deadband_sps,
                args.step, args.dir, last_dir, args.dir_invert,
                args.dir_blanking_us, args.dir_setup_us,
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

            # Free CBs before allocating next wave
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
            snap       = [list(c) for c in lms_coeffs]
            final_freqs = list(lms_freqs)
        print("\nFinal LMS coefficients:")
        for i, f_init in enumerate(F_init):
            C, S   = snap[i]
            A      = math.sqrt(C**2 + S**2)
            phi    = math.degrees(math.atan2(C, S))
            f_final = final_freqs[i]
            print(f"  tone{i+1}  {f_final:.3f}Hz (init {f_init:.3f}Hz)  "
                  f"A={A:.3f}mm  φ={phi:+.1f}°")

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

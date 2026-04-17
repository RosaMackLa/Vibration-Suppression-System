#!/usr/bin/env python3
"""
dual_wavestepper.py  —  Two-axis sinusoidal motion patterns
═══════════════════════════════════════════════════════════════

Drives X and Y stepper axes simultaneously to trace geometric
patterns: circles, Lissajous figures, figure-8, frequency sweeps,
and independent per-axis motion.

HARDWARE
────────
  X axis:  STEP BCM 13  DIR BCM 16  ENA BCM 25
  Y axis:  STEP BCM 19  DIR BCM 20  ENA BCM 21

PATTERNS
────────
  circle      X = A·cos(2π·f·t),      Y = A·sin(2π·f·t)
              [Y is 90° behind X → traces a circle]

  lissajous   X = Ax·cos(2π·fx·t),    Y = Ay·cos(2π·fy·t + φ)
              Classic Lissajous figures when fx:fy is rational.
              --freq_x 3 --freq_y 2 --phase_deg 90  → trifolium

  figure8     X = A·cos(2π·f·t),      Y = A·sin(2π·2f·t)
              fy = 2·fx → figure-eight (horizontal)

  sweep       Circle that sweeps frequency from --sweep_start to
              --sweep_end over the run duration.

  independent Each axis runs its own --freq_x / --freq_y and
              --amp_x / --amp_y independently. Useful to show
              each axis working before combining.

PULSE MERGING ARCHITECTURE
──────────────────────────
pigpio has a single DMA waveform engine.  X and Y cannot run
separate wave loops; all GPIO transitions must live in one wave.

  1. generate_axis_pulses() computes step events for ONE axis,
     returning a list of (abs_time_us, gpio_on_mask, gpio_off_mask).
  2. merge_pulse_events() merges X + Y event lists, sorts by time,
     and collapses simultaneous events by OR-ing their GPIO masks.
  3. events_to_pigpio() converts back to pigpio.pulse objects.

This approach guarantees both axes stay in perfect phase lock —
they share one timeline, one DMA engine, and one clock.

USAGE
─────
  # Restart pigpiod first:
  sudo pigpiod -s 2 -b 4096

  # Circle at 5 Hz, 15mm amplitude, 30 seconds:
  sudo chrt -f 50 python3 dual_wavestepper.py \\
      --pattern circle --freq 5 --amp 15 --duration 30

  # Lissajous 3:2 (trifolium-ish):
  sudo chrt -f 50 python3 dual_wavestepper.py \\
      --pattern lissajous --freq_x 3 --freq_y 2 \\
      --amp_x 20 --amp_y 15 --phase_deg 90 --duration 30

  # Figure-8:
  sudo chrt -f 50 python3 dual_wavestepper.py \\
      --pattern figure8 --freq 4 --amp 15 --duration 30

  # Frequency sweep: circle growing from 1 Hz → 8 Hz:
  sudo chrt -f 50 python3 dual_wavestepper.py \\
      --pattern sweep --sweep_start 1.0 --sweep_end 8.0 \\
      --amp 20 --duration 60

  # Independent axes (X at 3 Hz, Y at 5 Hz):
  sudo chrt -f 50 python3 dual_wavestepper.py \\
      --pattern independent --freq_x 3 --freq_y 5 \\
      --amp_x 15 --amp_y 15 --duration 30
"""

import argparse
import math
import sys
import time
import threading

import pigpio

# ─────────────────────────── Hardware constants ─────────────────────────────

# X axis (existing)
X_STEP = 13
X_DIR  = 16
X_ENA  = 25

# Y axis (new)
Y_STEP = 19
Y_DIR  = 20
Y_ENA  = 21

# Shared backstop button (all physical endstops OR'd to one pin)
BUTTON_PIN = 17   # BCM 17, externally conditioned; active HIGH (1 = any button pressed)

# Mechanical constants (shared between axes — same belt/pulley spec)
DEFAULT_PULLEY_D_MM    = 11.70
DEFAULT_PULSES_PER_REV = 3200
HALF_TRAVEL_MM         = 75.0

# Pulse timing (µs)
STEP_WIDTH_US   = 2       # STEP pin high time
DIR_SETUP_US    = 20      # DIR settle time before first step after direction change
DIR_BLANKING_US = 200     # Min time between last step of old dir and first of new

# Chunk / sample config
CHUNK_MS  = 50            # DMA chunk duration in milliseconds
SAMPLE_HZ = 4000          # Trajectory sample rate within each chunk


def k_steps_per_mm(pulley_d_mm=DEFAULT_PULLEY_D_MM,
                   pulses_per_rev=DEFAULT_PULSES_PER_REV):
    circumference = math.pi * pulley_d_mm
    return pulses_per_rev / circumference


# ──────────────────────────── Auto-centering ────────────────────────────────

def _btn(pi, active_low=False):
    """Return True if any backstop button is currently pressed."""
    return (pi.read(BUTTON_PIN) == 0) if active_low else (pi.read(BUTTON_PIN) == 1)


def _step_once(pi, step_pin, dir_pin, direction, dir_invert,
               last_actual_dir, period_us):
    """
    Output one step pulse and observe the step period.

    direction      : +1 or -1 in logical (pre-invert) space
    last_actual_dir: the last DIR-pin state (+1 or -1 in hardware space)
    Returns new last_actual_dir.
    """
    actual_dir = direction * (-1 if dir_invert else 1)
    if actual_dir != last_actual_dir:
        pi.write(dir_pin, 1 if actual_dir > 0 else 0)
        time.sleep(DIR_SETUP_US * 1e-6)
    pi.write(step_pin, 1)
    time.sleep(STEP_WIDTH_US * 1e-6)
    pi.write(step_pin, 0)
    remainder = period_us - STEP_WIDTH_US
    if remainder > 0:
        time.sleep(remainder * 1e-6)
    return actual_dir


def _period_us(step_idx, total_steps, max_sps, min_sps=800, ramp_frac=0.20):
    """
    Trapezoidal velocity profile — ramp up for the first ramp_frac of
    steps, cruise, then ramp back down.  Prevents stall on cold start.
    """
    ramp = max(1, int(total_steps * ramp_frac))
    if step_idx < ramp:
        sps = min_sps + (max_sps - min_sps) * (step_idx / ramp)
    elif step_idx > total_steps - ramp:
        sps = min_sps + (max_sps - min_sps) * ((total_steps - step_idx) / ramp)
    else:
        sps = max_sps
    return int(1e6 / max(sps, 1.0))


def autocenter_axis(pi, step_pin, dir_pin, axis_name,
                    k, dir_invert, approach_dir,
                    approach_sps, return_sps,
                    center_mm, active_low=True):
    """
    Find one axis wall then back off to centre.

    Algorithm
    ─────────
    1. If button already pressed → skip approach, go straight to back-off.
    2. Otherwise step slowly in `approach_dir` (+1 or -1) until button fires
       OR the safety step limit is reached.
    3. Pause briefly, then drive in the opposite direction by `center_mm`
       using a trapezoidal velocity profile.

    Parameters
    ──────────
    approach_dir : +1 → move carriage toward the positive-direction wall first
                   -1 → move toward the negative-direction wall first
    center_mm    : distance to back off after touching the wall.
                   Set this to the distance from the wall to mechanical centre
                   (typically HALF_TRAVEL_MM).
    """
    MAX_APPROACH_STEPS = int((HALF_TRAVEL_MM + 15.0) * k)   # hard safety limit
    approach_period_us = int(1e6 / max(approach_sps, 1.0))
    return_steps       = int(center_mm * k)

    print(f"\n  [{axis_name}] ── Centering ──────────────────────────────")

    # ── Phase 1: approach wall ──────────────────────────────────────────────
    if _btn(pi, active_low):
        print(f"  [{axis_name}] Button already pressed — skipping approach.")
        steps_taken = 0
    else:
        print(f"  [{axis_name}] Approaching wall at {approach_sps} sps "
              f"(dir={'+ ' if approach_dir > 0 else '- '}logical)...")
        last_dir   = 0   # unknown at start; _step_once handles DIR setup
        steps_taken = 0

        # Set DIR before loop to avoid first-step direction jitter
        actual_dir = approach_dir * (-1 if dir_invert else 1)
        pi.write(dir_pin, 1 if actual_dir > 0 else 0)
        time.sleep(DIR_SETUP_US * 1e-6)
        last_dir = actual_dir

        for _ in range(MAX_APPROACH_STEPS):
            if _btn(pi, active_low):
                break
            last_dir = _step_once(pi, step_pin, dir_pin,
                                   approach_dir, dir_invert,
                                   last_dir, approach_period_us)
            steps_taken += 1
        else:
            print(f"  [{axis_name}] WARNING: hit step limit ({MAX_APPROACH_STEPS/k:.0f} mm) "
                  "without triggering button.  Check button wiring / active level.")
            return

        print(f"  [{axis_name}] Button triggered after {steps_taken} steps "
              f"({steps_taken/k:.1f} mm travel).")

    # Brief pause so the carriage is fully at rest before reversing
    time.sleep(0.25)

    # ── Phase 2: back off to centre ─────────────────────────────────────────
    return_dir = -approach_dir
    print(f"  [{axis_name}] Backing off {center_mm:.1f} mm "
          f"({return_steps} steps) at up to {return_sps} sps...")

    actual_dir = return_dir * (-1 if dir_invert else 1)
    pi.write(dir_pin, 1 if actual_dir > 0 else 0)
    time.sleep(DIR_SETUP_US * 1e-6)
    last_dir = actual_dir

    for i in range(return_steps):
        # Safety check: shouldn't hit a button during return, but stop if so
        if _btn(pi, active_low) and i > 200:
            print(f"  [{axis_name}] Button during return at step {i} — stopping.")
            break
        period = _period_us(i, return_steps, return_sps)
        last_dir = _step_once(pi, step_pin, dir_pin,
                               return_dir, dir_invert,
                               last_dir, period)

    print(f"  [{axis_name}] Centred ✓")


def autocenter(pi, args, k):
    """
    Centre X axis, then Y axis.
    Both drivers stay enabled (holding current) throughout so neither
    axis drifts while the other is being centred.
    """
    print("\n" + "═" * 56)
    print("  AUTO-CENTERING")
    print("═" * 56)
    print("  Drivers enabled on both axes to prevent drift.")
    print("  Move carriages clear of walls before starting if needed.")
    print()

    active_low = args.button_active_low

    autocenter_axis(
        pi, X_STEP, X_DIR, "X",
        k, args.dir_invert_x,
        approach_dir  = args.approach_dir_x,
        approach_sps  = args.approach_sps,
        return_sps    = args.return_sps,
        center_mm     = args.center_mm,
        active_low    = active_low,
    )

    # Small pause between axes so vibrations from X settle
    time.sleep(0.5)

    autocenter_axis(
        pi, Y_STEP, Y_DIR, "Y",
        k, args.dir_invert_y,
        approach_dir  = args.approach_dir_y,
        approach_sps  = args.approach_sps,
        return_sps    = args.return_sps,
        center_mm     = args.center_mm,
        active_low    = active_low,
    )

    print("\n  Both axes centred.  Starting pattern in 1 s...")
    time.sleep(1.0)


# ──────────────────────── Single-axis pulse generation ──────────────────────

def generate_axis_pulses(A_mm, freq_hz, phase_rad, chunk_s,
                          k, step_pin, dir_pin,
                          last_dir, dir_invert,
                          step_phi_carry,    # float phase accumulator (radians)
                          x_est_steps,       # float: current step position
                          max_sps):
    """
    Generate pulse EVENTS for one axis over `chunk_s` seconds.

    The trajectory is:  x(t) = A_mm · cos(2π·freq·t + step_phi_carry)

    Returns
    ───────
    events     : list of [abs_time_us, gpio_on_mask, gpio_off_mask]
    last_dir   : final direction state (1 or -1)
    step_phi_carry : updated phase accumulator
    x_est_steps    : updated step position (float)
    """
    N        = max(1, int(chunk_s * SAMPLE_HZ))
    dt       = chunk_s / N          # seconds per sample
    dt_us    = int(dt * 1e6)        # µs per sample

    omega    = 2.0 * math.pi * freq_hz
    frac_carry = x_est_steps - round(x_est_steps)
    int_steps  = round(x_est_steps)

    events = []   # [abs_time_us, gpio_on_mask, gpio_off_mask]
    cur_phase = step_phi_carry
    abs_t_us  = 0

    # Step period floor = 1/max_sps converted to µs
    min_step_period_us = max(int(1e6 / max_sps), STEP_WIDTH_US + 4)

    last_step_time_us = -min_step_period_us  # allows immediate first step

    for n in range(N):
        target_mm    = A_mm * math.cos(cur_phase)
        target_float = target_mm * k + frac_carry
        delta_steps  = int(round(target_float)) - int_steps

        if delta_steps != 0:
            direction = 1 if delta_steps > 0 else -1
            actual_dir = direction if not dir_invert else -direction
            dir_pin_high = actual_dir > 0

            for _ in range(abs(delta_steps)):
                step_time = abs_t_us

                # Rate-limit: enforce min period between steps
                if step_time - last_step_time_us < min_step_period_us:
                    step_time = last_step_time_us + min_step_period_us

                # Direction change: need DIR setup time before step
                if actual_dir != (1 if last_dir > 0 else -1):
                    dir_time = max(0, step_time - DIR_SETUP_US)
                    dir_mask = 1 << dir_pin
                    if dir_pin_high:
                        events.append([dir_time, dir_mask, 0])
                    else:
                        events.append([dir_time, 0, dir_mask])
                    step_time = max(step_time, dir_time + DIR_SETUP_US)
                    last_dir  = actual_dir

                # STEP high
                events.append([step_time, 1 << step_pin, 0])
                # STEP low
                events.append([step_time + STEP_WIDTH_US, 0, 1 << step_pin])
                last_step_time_us = step_time

                int_steps += direction

        frac_carry  = target_float - (int_steps - round(x_est_steps) + round(x_est_steps))
        cur_phase  += omega * dt
        abs_t_us   += dt_us

    # Normalise phase to [0, 2π)
    new_phase = math.fmod(cur_phase, 2.0 * math.pi)
    if new_phase < 0:
        new_phase += 2.0 * math.pi

    return events, last_dir, new_phase, float(int_steps)


# ──────────────────────────── Pulse merging ─────────────────────────────────

def merge_and_build_wave(events_x, events_y, chunk_s):
    """
    Merge X + Y event lists into a single sorted pigpio.pulse list.

    Simultaneous events (same abs_time_us) get their GPIO masks OR'd
    together so they become one atomic pigpio transition.  X and Y use
    different GPIO pins so masks never conflict.
    """
    all_events = events_x + events_y

    if not all_events:
        # Empty chunk — return a single idle pulse spanning the chunk
        return [pigpio.pulse(0, 0, int(chunk_s * 1e6))]

    # Sort by absolute time
    all_events.sort(key=lambda e: e[0])

    # Collapse events at the same timestamp
    merged = []
    for ev in all_events:
        t, on, off = ev
        if merged and merged[-1][0] == t:
            merged[-1][1] |= on
            merged[-1][2] |= off
        else:
            merged.append([t, on, off])

    # Convert absolute times → relative delays (pigpio.pulse delay follows
    # the transition, i.e. how long to hold the state before the next pulse)
    pulses = []
    chunk_us = int(chunk_s * 1e6)
    prev_t = 0
    for t, on, off in merged:
        delay = t - prev_t
        if delay < 0:
            delay = 0
        pulses.append(pigpio.pulse(on, off, delay))
        prev_t = t

    # Pad to fill the full chunk duration
    remaining = chunk_us - prev_t
    if remaining > 0:
        pulses.append(pigpio.pulse(0, 0, remaining))

    return pulses


# ──────────────────────────────── Main ──────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=__doc__,
    )

    p.add_argument("--pattern", choices=["circle", "lissajous", "figure8",
                                          "sweep", "independent"],
                   default="circle", help="Motion pattern")

    # Shared amplitude / frequency
    p.add_argument("--amp",      type=float, default=15.0,
                   help="Amplitude [mm] for circle / figure8 / sweep")
    p.add_argument("--freq",     type=float, default=5.0,
                   help="Base frequency [Hz] for circle / figure8 / sweep")

    # Per-axis overrides
    p.add_argument("--amp_x",   type=float, default=None, help="X amplitude [mm]")
    p.add_argument("--amp_y",   type=float, default=None, help="Y amplitude [mm]")
    p.add_argument("--freq_x",  type=float, default=None, help="X frequency [Hz]")
    p.add_argument("--freq_y",  type=float, default=None, help="Y frequency [Hz]")

    # Lissajous phase
    p.add_argument("--phase_deg", type=float, default=90.0,
                   help="Phase offset Y relative to X [deg] (lissajous)")

    # Sweep
    p.add_argument("--sweep_start", type=float, default=1.0,
                   help="Start frequency for sweep [Hz]")
    p.add_argument("--sweep_end",   type=float, default=8.0,
                   help="End frequency for sweep [Hz]")

    # Run control
    p.add_argument("--duration",  type=float, default=30.0, help="Run duration [s]")
    p.add_argument("--max_sps",   type=float, default=50000.0,
                   help="Max step rate per axis [steps/s]")

    # Hardware
    p.add_argument("--pulley_d_mm",    type=float, default=DEFAULT_PULLEY_D_MM)
    p.add_argument("--pulses_per_rev", type=float, default=DEFAULT_PULSES_PER_REV)
    p.add_argument("--dir_invert_x",   action="store_true",
                   help="Invert X direction")
    p.add_argument("--dir_invert_y",   action="store_true",
                   help="Invert Y direction")

    # Auto-centering
    p.add_argument("--autocenter",         action="store_true",
                   help="Run auto-centering routine before starting pattern")
    p.add_argument("--approach_sps",       type=float, default=600.0,
                   help="Step rate during wall approach [sps]  (default 600 — slow & gentle)")
    p.add_argument("--return_sps",         type=float, default=4000.0,
                   help="Max step rate during back-off to centre [sps]  (default 4000)")
    p.add_argument("--center_mm",          type=float, default=50.0,
                   help="Distance to back off after wall touch [mm]  (default 50)")
    p.add_argument("--approach_dir_x",     type=int,   default=-1, choices=[1, -1],
                   help="Logical direction to approach wall on X  (+1 or -1, default -1)")
    p.add_argument("--approach_dir_y",     type=int,   default=-1, choices=[1, -1],
                   help="Logical direction to approach wall on Y  (+1 or -1, default -1)")
    p.add_argument("--button_active_low", action="store_true",
                   help="Button pin is active LOW (default: active HIGH)")

    return p.parse_args()


def pattern_params(args, t_elapsed, total_duration):
    """
    Return (Ax, Ay, fx, fy, phase_x, phase_y) for the current time.
    All returned phases are absolute starting phases for the current chunk.
    (The step_phi_carry in the drive loop handles the actual accumulation.)
    """
    pat = args.pattern

    if pat == "circle":
        A  = args.amp
        f  = args.freq
        return A, A, f, f, 0.0, -math.pi / 2   # Y lags X by 90° → circle

    if pat == "lissajous":
        Ax = args.amp_x if args.amp_x is not None else args.amp
        Ay = args.amp_y if args.amp_y is not None else args.amp
        fx = args.freq_x if args.freq_x is not None else args.freq
        fy = args.freq_y if args.freq_y is not None else args.freq
        phi = math.radians(args.phase_deg)
        return Ax, Ay, fx, fy, 0.0, phi

    if pat == "figure8":
        A = args.amp
        f = args.freq
        # X at f, Y at 2f with 90° offset → figure-8
        return A, A, f, 2.0 * f, 0.0, -math.pi / 2

    if pat == "sweep":
        A  = args.amp
        progress = t_elapsed / max(total_duration, 1.0)
        f  = args.sweep_start + progress * (args.sweep_end - args.sweep_start)
        f  = max(0.1, f)
        return A, A, f, f, 0.0, -math.pi / 2

    if pat == "independent":
        Ax = args.amp_x if args.amp_x is not None else args.amp
        Ay = args.amp_y if args.amp_y is not None else args.amp
        fx = args.freq_x if args.freq_x is not None else args.freq
        fy = args.freq_y if args.freq_y is not None else args.freq
        return Ax, Ay, fx, fy, 0.0, 0.0

    raise ValueError(f"Unknown pattern: {pat}")


def main():
    args = parse_args()
    chunk_s = CHUNK_MS / 1000.0
    k = k_steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    print("═" * 56)
    print("  DUAL-AXIS WAVESTEPPER")
    print("═" * 56)
    print(f"  Pattern   : {args.pattern}")
    print(f"  Duration  : {args.duration:.1f} s")
    print(f"  steps/mm  : {k:.4f}")
    print(f"  ±travel   : {HALF_TRAVEL_MM:.0f} mm  (~{int(HALF_TRAVEL_MM*k)} steps)")
    print(f"  max_sps   : {args.max_sps:.0f}")
    print()

    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("ERROR: pigpiod not running.  sudo pigpiod -s 2 -b 4096")

    for pin in [X_STEP, X_DIR, X_ENA, Y_STEP, Y_DIR, Y_ENA]:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)

    # Enable both drivers (active low on most drivers; adjust if needed)
    pi.write(X_ENA, 1)
    pi.write(Y_ENA, 1)

    # Set up button pin as input with pull-up
    pi.set_mode(BUTTON_PIN, pigpio.INPUT)
    pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_UP)

    # ── Auto-centering ────────────────────────────────────────────────────────
    if args.autocenter:
        autocenter(pi, args, k)

    pi.wave_clear()

    # State for each axis
    x_last_dir   = 1
    y_last_dir   = 1
    x_phi        = 0.0    # phase accumulator
    y_phi        = 0.0
    x_est        = 0.0    # estimated position in steps (float)
    y_est        = 0.0

    # Set initial Y phase from pattern (circle needs 90° offset from start)
    Ax0, Ay0, fx0, fy0, phi_x0, phi_y0 = pattern_params(args, 0.0, args.duration)
    x_phi = phi_x0
    y_phi = phi_y0

    print("  Starting motors.  Ctrl-C to stop.\n")
    print(f"  {'t':>7s}  {'X_est':>8s}  {'Y_est':>8s}  {'fx':>6s}  {'fy':>6s}")
    print(f"  {'':─>7s}  {'':─>8s}  {'':─>8s}  {'':─>6s}  {'':─>6s}")

    t_start = time.monotonic()
    t_chunk_wall = t_start

    # ── Build first chunk ──
    t_elapsed = 0.0
    Ax, Ay, fx, fy, _, _ = pattern_params(args, t_elapsed, args.duration)

    evx, x_last_dir, x_phi, x_est = generate_axis_pulses(
        Ax, fx, x_phi, chunk_s, k,
        X_STEP, X_DIR, x_last_dir, args.dir_invert_x, x_phi, x_est, args.max_sps)

    evy, y_last_dir, y_phi, y_est = generate_axis_pulses(
        Ay, fy, y_phi, chunk_s, k,
        Y_STEP, Y_DIR, y_last_dir, args.dir_invert_y, y_phi, y_est, args.max_sps)

    pulses_c = merge_and_build_wave(evx, evy, chunk_s)
    pi.wave_add_generic(pulses_c)
    wave_c = pi.wave_create()
    pi.wave_send_once(wave_c)

    chunk_count = 0

    try:
        while True:
            t_elapsed = time.monotonic() - t_start

            if t_elapsed >= args.duration:
                # Let the last wave finish
                while pi.wave_tx_busy():
                    time.sleep(0.001)
                break

            # ── Prepare next chunk ──
            Ax, Ay, fx, fy, _, _ = pattern_params(args, t_elapsed, args.duration)

            evx_n, x_last_dir_n, x_phi_n, x_est_n = generate_axis_pulses(
                Ax, fx, x_phi, chunk_s, k,
                X_STEP, X_DIR, x_last_dir, args.dir_invert_x, x_phi, x_est, args.max_sps)

            evy_n, y_last_dir_n, y_phi_n, y_est_n = generate_axis_pulses(
                Ay, fy, y_phi, chunk_s, k,
                Y_STEP, Y_DIR, y_last_dir, args.dir_invert_y, y_phi, y_est, args.max_sps)

            pulses_n = merge_and_build_wave(evx_n, evy_n, chunk_s)

            # ── Wait for current chunk to finish ──
            sleep_s = chunk_s - (time.monotonic() - t_chunk_wall) - 0.008
            if sleep_s > 0:
                time.sleep(sleep_s)
            while pi.wave_tx_busy():
                time.sleep(0.0001)

            # ── Travel limit check ──
            x_mm = x_est_n / k
            y_mm = y_est_n / k
            if abs(x_mm) > HALF_TRAVEL_MM or abs(y_mm) > HALF_TRAVEL_MM:
                print(f"\n  TRAVEL LIMIT: X={x_mm:+.1f}mm  Y={y_mm:+.1f}mm — stopping.")
                break

            # ── Swap wave ──
            pi.wave_delete(wave_c)
            pi.wave_add_generic(pulses_n)
            wave_n = pi.wave_create()
            pi.wave_send_once(wave_n)
            t_chunk_wall = time.monotonic()

            wave_c     = wave_n
            x_last_dir = x_last_dir_n
            y_last_dir = y_last_dir_n
            x_phi      = x_phi_n
            y_phi      = y_phi_n
            x_est      = x_est_n
            y_est      = y_est_n

            chunk_count += 1
            if chunk_count % 4 == 0:
                print(f"  {t_elapsed:>7.1f}s  "
                      f"{x_est/k:>+7.1f}mm  "
                      f"{y_est/k:>+7.1f}mm  "
                      f"{fx:>5.3f}Hz  "
                      f"{fy:>5.3f}Hz")

    except KeyboardInterrupt:
        print("\n  Stopped by user.")

    finally:
        try:
            pi.wave_tx_stop()
            pi.wave_clear()
        except Exception:
            pass
        for pin in [X_STEP, X_DIR, X_ENA, Y_STEP, Y_DIR, Y_ENA]:
            try:
                pi.write(pin, 0)
            except Exception:
                pass
        pi.stop()
        print(f"\n  Final: X={x_est/k:+.1f}mm  Y={y_est/k:+.1f}mm  "
              f"({chunk_count} chunks)\n  Done.")


if __name__ == "__main__":
    main()

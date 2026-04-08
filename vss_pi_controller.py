#!/usr/bin/env python3
"""
vss_pi_controller.py — VSS velocity-error PI vibration suppression

CONTROL LAW
───────────
  Structural velocity is estimated by leaky integration of IMU x-axis
  acceleration:

      v_est[n] = v_est[n-1] * (1 - dt/τ) + a_imu[n] * dt

  The leak term dt/τ (controlled by --tau_vel) prevents unbounded drift
  from accelerometer DC bias.  Shorter τ → faster DC rejection but more
  noise sensitivity.  Longer τ → smoother estimate but slower to reject DC.

  PI error (target = zero structural velocity):

      e_v(t) = v_est(t)

  PI output — carriage velocity command (mm/s):

      u(t) = −[ Kp · e_v(t) + Ki · ∫e_v dt ]

  The negative sign commands the carriage to oppose structural velocity:
  when the structure moves in the +x direction the carriage is driven in
  the −x direction, creating an inertial reaction force that damps the
  motion.  If the sign is wrong for your physical layout, use --dir_invert.

  Anti-windup: the integral is clamped to ±(max_vel_mm / Ki) so the
  integrator alone can never saturate the output.

ACTUATION
─────────
  The PI output is a signed velocity command (mm/s).  At each DMA chunk
  boundary (~chunk_ms period) the command is converted to a constant step
  rate for that chunk.  chunk_ms defaults to 10 ms (100 Hz update rate),
  which gives ~11 updates per cycle at 9 Hz.  Reduce further if you need
  more bandwidth, but below ~5 ms the chunk computation overhead becomes
  significant on the Pi.

TUNING GUIDE
────────────
  Start with Ki=0 and increase Kp only:
    Kp too low  → carriage barely moves, no damping effect
    Kp too high → carriage oscillates independently of disturbance
  Typical starting point: Kp=0.5–2.0 depending on carriage mass vs structure.

  Once Kp gives visible damping, add Ki slowly (start at Kp/20):
    Ki too high → slow low-frequency oscillation, windup

  tau_vel:
    0.1–0.2 s → fast response, noisy velocity estimate
    0.3–0.5 s → balanced (good starting point)
    1.0 s+    → smooth but sluggish at tracking velocity changes

  NOTE: --simulate only tests that the software runs without crashing.
  It does not include a plant model so PI cancellation will not be visible
  in simulation — the carriage moves but has no effect on the simulated error.

USAGE
─────
  # Proportional only, start here:
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 vss_pi_controller.py \\
      --kp 1.0 --ki 0.0 --tau_vel 0.3 \\
      --max_sps 50000 --max_vel_mm 50 --cancel_dur 120 --dir_invert

  # Add integral once Kp is tuned:
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 vss_pi_controller.py \\
      --kp 1.0 --ki 0.05 --tau_vel 0.3 \\
      --max_sps 50000 --max_vel_mm 50 --cancel_dur 120 --dir_invert
"""

import argparse
import math
import time
import threading
import sys
import random

import numpy as np
import matplotlib
matplotlib.use('Agg')
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
DEFAULT_STEP_BCM       = 13
DEFAULT_DIR_BCM        = 16
PULSE_HIGH_US          = 5
ENABLE_PIN             = 25

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
    """Read x-axis acceleration from LSM6DSO32 [m/s²]."""
    return _get_imu().acceleration[0]


def sim_disturbance(t: float) -> float:
    """Synthetic disturbance for --simulate mode (no plant model)."""
    return (0.17 * math.sin(2 * math.pi * 8.8 * t) +
            0.003 * (2 * random.random() - 1))

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
        description="VSS velocity-error PI controller",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    hw = p.add_argument_group("Hardware")
    hw.add_argument("--step",            type=int,   default=DEFAULT_STEP_BCM)
    hw.add_argument("--dir",             type=int,   default=DEFAULT_DIR_BCM)
    hw.add_argument("--dir_invert",      action="store_true")
    hw.add_argument("--pulley_d_mm",     type=float, default=DEFAULT_PULLEY_D_MM)
    hw.add_argument("--pulses_per_rev",  type=float, default=DEFAULT_PULSES_PER_REV)
    hw.add_argument("--max_sps",         type=float, default=50000.0)
    hw.add_argument("--deadband_sps",    type=float, default=20.0)
    hw.add_argument("--chunk_ms",        type=float, default=10.0,
                    help="DMA chunk duration (ms). 10ms = 100Hz waveform update rate.")
    hw.add_argument("--dir_setup_us",    type=int,   default=20)
    hw.add_argument("--dir_blanking_us", type=int,   default=200)

    pi_g = p.add_argument_group("PI Controller")
    pi_g.add_argument("--kp",         type=float, default=1.0,
                      help="Proportional gain [(mm/s) / (m/s)]")
    pi_g.add_argument("--ki",         type=float, default=0.0,
                      help="Integral gain [(mm/s) / (m/s·s)].  Start at 0.")
    pi_g.add_argument("--tau_vel",    type=float, default=0.3,
                      help="Leaky integrator time constant (s) for velocity estimate.")
    pi_g.add_argument("--max_vel_mm", type=float, default=50.0,
                      help="Carriage velocity command clamp (mm/s).")
    pi_g.add_argument("--control_fs", type=float, default=800.0,
                      help="PI loop sample rate (Hz).")

    sim = p.add_argument_group("Simulation")
    sim.add_argument("--simulate", action="store_true",
                     help="Use synthetic IMU signal (no plant model — "
                          "tests software only, not control performance).")

    log = p.add_argument_group("Logging & Plotting")
    log.add_argument("--log_hz",   type=float, default=100.0,
                     help="Rate at which PI state is logged [Hz].")
    log.add_argument("--no_plot",  action="store_true",
                     help="Skip plot generation after run.")
    log.add_argument("--plot_out", type=str,   default="vss_pi_plot.png",
                     help="Output filename for the post-run plot.")

    p.add_argument("--cancel_dur",       type=float, default=0.0,
                   help="Run duration (s), 0=forever.")
    p.add_argument("--status_hz",        type=float, default=2.0)
    p.add_argument("--no_travel_safety", action="store_true")

    return p.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
# DMA chunk — constant velocity
# ══════════════════════════════════════════════════════════════════════════════

def compute_chunk_vel(vel_cmd_mm_s, chunk_s, k,
                      max_sps, deadband_sps,
                      step_pin, dir_pin, last_dir, dir_invert,
                      dir_blanking_us, dir_setup_us):
    """
    Generate a constant-velocity DMA chunk spanning exactly chunk_s seconds.

    vel_cmd_mm_s : signed carriage velocity command (mm/s).
                   Sign determines direction; magnitude determines step rate.

    The chunk is always exactly chunk_s long regardless of how many steps
    fit — remaining time after the last step is padded with a delay pulse.
    This ensures the double-buffer timing stays accurate.

    Returns (pulses, end_dir, net_steps).
    net_steps is signed: positive = physical positive direction.
    """
    STEP_BIT      = 1 << step_pin
    DIR_BIT       = 1 << dir_pin
    MIN_PERIOD_US = PULSE_HIGH_US + 1

    chunk_us    = max(1, int(chunk_s * 1e6))
    sps         = min(abs(vel_cmd_mm_s) * k, max_sps)
    pulses      = []
    current_dir = last_dir
    net_steps   = 0
    elapsed_us  = 0

    # Below deadband: emit a single delay to fill the chunk, no motion
    if sps < deadband_sps:
        pulses.append(pigpio.pulse(0, 0, chunk_us))
        return pulses, current_dir, 0

    logical_dir = (1 if vel_cmd_mm_s >= 0.0 else 0) ^ int(dir_invert)

    # Direction change — insert blanking + setup delays, track elapsed time
    if logical_dir != current_dir:
        if dir_blanking_us > 0:
            d = min(dir_blanking_us, chunk_us - elapsed_us)
            if d > 0:
                pulses.append(pigpio.pulse(0, 0, d))
                elapsed_us += d
        d = min(dir_setup_us, chunk_us - elapsed_us)
        if d > 0:
            pulses.append(pigpio.pulse(
                DIR_BIT if logical_dir else 0,
                0       if logical_dir else DIR_BIT,
                d,
            ))
            elapsed_us += d
            current_dir = logical_dir

    # Fill remaining chunk time with constant-rate steps
    period_us = max(MIN_PERIOD_US, int(round(1e6 / sps)))

    while elapsed_us + period_us <= chunk_us:
        pulses.append(pigpio.pulse(STEP_BIT, 0, PULSE_HIGH_US))
        pulses.append(pigpio.pulse(0, STEP_BIT, period_us - PULSE_HIGH_US))
        elapsed_us += period_us
        net_steps  += 1

    # Pad remainder so chunk is exactly chunk_us long
    remainder = chunk_us - elapsed_us
    if remainder > 0:
        pulses.append(pigpio.pulse(0, 0, remainder))

    # Sign net_steps by physical direction
    phys_dir   = current_dir ^ int(dir_invert)
    net_steps  = net_steps if phys_dir else -net_steps

    return pulses, current_dir, net_steps


def wave_dur_s(pulses) -> float:
    return sum(p.delay for p in pulses) / 1e6

# ══════════════════════════════════════════════════════════════════════════════
# PI control thread
# ══════════════════════════════════════════════════════════════════════════════

def pi_thread_fn(pi_state, pi_lock, t_start, args, running, pi_stats, log_buf):
    """
    Runs at ~control_fs Hz.

    STEP 1 — Leaky velocity integration:
        v_est[n] = v_est[n-1] * (1 - dt/τ) + a_imu[n] * dt

    STEP 2 — PI control (error = structural velocity, target = 0):
        integral += e_v * dt              (anti-windup clamped)
        u = −(Kp·e_v + Ki·integral)       (negative = oppose velocity)
        u = clamp(u, ±max_vel_mm)

    STEP 3 — Publish u to pi_state['vel_cmd'] under pi_lock so the main
              DMA loop can snapshot it at each chunk boundary.

    Anti-windup clamp:
        |integral| ≤ max_vel_mm / Ki
    This ensures the integrator alone can never saturate the output,
    preventing slow recovery after the carriage hits a travel limit.
    """
    dt      = 1.0 / args.control_fs
    leak    = dt / max(args.tau_vel, 1e-6)
    kp      = args.kp
    ki      = args.ki
    max_vel = args.max_vel_mm

    v_est    = 0.0
    integral = 0.0
    max_integral = (max_vel / ki) if ki > 1e-12 else float('inf')

    log_every  = max(1, int(args.control_fs / args.log_hz))
    log_ticker = 0
    next_t     = time.monotonic()

    while running[0]:
        now = time.monotonic()
        if now < next_t:
            time.sleep(next_t - now)
            now = time.monotonic()

        # Read IMU acceleration
        if args.simulate:
            a = sim_disturbance(now - t_start)
        else:
            a = hw_read_accel()

        # Leaky velocity integration
        v_est = v_est * (1.0 - leak) + a * dt

        # PI on velocity error
        e_v      = v_est
        integral = max(-max_integral, min(max_integral, integral + e_v * dt))
        vel_cmd  = -(kp * e_v + ki * integral)
        vel_cmd  = max(-max_vel, min(max_vel, vel_cmd))

        with pi_lock:
            pi_state['vel_cmd'] = vel_cmd
            pi_state['v_est']   = v_est
            pi_state['a_imu']   = a

        pi_stats['v_est']   = v_est
        pi_stats['vel_cmd'] = vel_cmd
        pi_stats['a_imu']   = a
        pi_stats['t']       = now - t_start

        log_ticker += 1
        if log_ticker >= log_every:
            log_ticker = 0
            log_buf['t'].append(now - t_start)
            log_buf['a'].append(a)
            log_buf['v'].append(v_est)
            log_buf['u'].append(vel_cmd)

        next_t += dt

# ══════════════════════════════════════════════════════════════════════════════
# Status thread
# ══════════════════════════════════════════════════════════════════════════════

def status_thread_fn(pi_hw, shared, shared_lock, args, running, pi_stats):
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
                pi_hw.get_current_tick()
            except Exception:
                pass
            next_ka = now + 2.0

        if now >= next_p:
            with shared_lock:
                sh = dict(shared)
            print(
                f"t={sh.get('t', 0):7.3f}s  "
                f"x_est={sh.get('x_est', 0):+7.2f}mm  "
                f"a_imu={pi_stats.get('a_imu', 0):+.4f} m/s²  "
                f"v_est={pi_stats.get('v_est', 0):+.5f} m/s  "
                f"vel_cmd={pi_stats.get('vel_cmd', 0):+6.2f} mm/s"
            )
            next_p += period

        time.sleep(0.01)

# ══════════════════════════════════════════════════════════════════════════════
# Post-run plot
# ══════════════════════════════════════════════════════════════════════════════

def plot_run(log_buf, args):
    """
    3-panel post-run plot:
      Panel 1 — IMU acceleration + 2s RMS envelope
      Panel 2 — Estimated structural velocity (the PI error signal) + 2s RMS
      Panel 3 — PI velocity command output
    """
    n = len(log_buf['t'])
    if n < 2:
        print("  Not enough data to plot.")
        return

    t = np.array(log_buf['t'])
    a = np.array(log_buf['a'])
    v = np.array(log_buf['v'])
    u = np.array(log_buf['u'])

    window = max(1, int(2.0 * args.log_hz))

    def rms_envelope(x):
        return np.array([
            np.sqrt(np.mean(x[max(0, i - window):i + 1] ** 2))
            for i in range(len(x))
        ])

    a_rms = rms_envelope(a)
    v_rms = rms_envelope(v)

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle(
        f"VSS PI Controller\n"
        f"Kp={args.kp}  Ki={args.ki}  τ_vel={args.tau_vel}s  "
        f"fs={args.control_fs:.0f}Hz  chunk={args.chunk_ms:.0f}ms  "
        f"dur={t[-1]:.1f}s",
        fontsize=10,
    )

    # Panel 1 — IMU acceleration
    ax = axes[0]
    ax.plot(t, a, alpha=0.25, color='gray', linewidth=0.4, label='a_imu(t)')
    ax.plot(t, a_rms, color='crimson', linewidth=1.5, label='a_rms (2s window)')
    ax.axhline(0, color='black', linewidth=0.5)
    ax.set_ylabel('Acceleration [m/s²]')
    ax.set_title('IMU Acceleration  (should fall if PI is damping effectively)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 2 — Estimated structural velocity (PI error signal)
    ax = axes[1]
    ax.plot(t, v * 1000, alpha=0.4, color='steelblue', linewidth=0.5,
            label='v_est(t)  [mm/s]')
    ax.plot(t, v_rms * 1000, color='navy', linewidth=1.5,
            label='v_rms (2s window)  [mm/s]')
    ax.axhline(0, color='black', linewidth=0.5)
    ax.set_ylabel('Velocity [mm/s]')
    ax.set_title('Estimated Structural Velocity  (PI error — target = 0)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel 3 — PI velocity command
    ax = axes[2]
    ax.plot(t, u, color='seagreen', linewidth=0.8, label='vel_cmd (mm/s)')
    ax.axhline( args.max_vel_mm, color='red', linestyle='--', linewidth=0.8,
                label=f'+clamp = {args.max_vel_mm} mm/s')
    ax.axhline(-args.max_vel_mm, color='red', linestyle='--', linewidth=0.8,
                label=f'−clamp = {-args.max_vel_mm} mm/s')
    ax.axhline(0, color='black', linewidth=0.5)
    ax.set_ylabel('Vel cmd [mm/s]')
    ax.set_xlabel('Time [s]')
    ax.set_title('PI Output — Carriage Velocity Command')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(args.plot_out, dpi=150)
    print(f"  Plot saved: {args.plot_out}")
    plt.close(fig)

# ══════════════════════════════════════════════════════════════════════════════
# Cancellation loop
# ══════════════════════════════════════════════════════════════════════════════

def run_cancellation(pi_hw, args, k):
    print(f"\n{'─'*56}")
    print(f"  PI VELOCITY CONTROLLER")
    print(f"{'─'*56}")
    print(f"  Kp={args.kp}  Ki={args.ki}  τ_vel={args.tau_vel}s")
    print(f"  max_vel_cmd={args.max_vel_mm} mm/s  "
          f"control_fs={args.control_fs:.0f}Hz  chunk={args.chunk_ms}ms")
    print(f"  Ctrl-C to stop.\n")

    pi_hw.wave_clear()

    pi_hw.set_mode(args.dir,   pigpio.OUTPUT)
    pi_hw.set_mode(args.step,  pigpio.OUTPUT)
    pi_hw.set_mode(ENABLE_PIN, pigpio.OUTPUT)
    pi_hw.write(ENABLE_PIN, 1)   # enable backstop interlock
    pi_hw.write(args.dir, 1)
    time.sleep(args.dir_setup_us / 1e6)

    t_wall_start = time.monotonic()

    pi_state    = {'vel_cmd': 0.0, 'v_est': 0.0, 'a_imu': 0.0}
    pi_lock     = threading.Lock()
    running     = [True]
    pi_stats    = {'v_est': 0.0, 'vel_cmd': 0.0, 'a_imu': 0.0, 't': 0.0}
    shared      = {'t': 0.0, 'x_est': 0.0}
    shared_lock = threading.Lock()
    log_buf     = {'t': [], 'a': [], 'v': [], 'u': []}
    chunk_s     = args.chunk_ms / 1000.0

    # ── Spawn control and status threads ──────────────────────────────────────
    ctrl_t = threading.Thread(
        target=pi_thread_fn,
        args=(pi_state, pi_lock, t_wall_start, args, running, pi_stats, log_buf),
        daemon=True, name="pi-ctrl",
    )
    ctrl_t.start()

    stat_t = threading.Thread(
        target=status_thread_fn,
        args=(pi_hw, shared, shared_lock, args, running, pi_stats),
        daemon=True, name="status",
    )
    stat_t.start()

    s_est      = 0.0
    last_dir   = 1
    t_end_wall = (t_wall_start + args.cancel_dur) if args.cancel_dur > 0 else None

    # ── Bootstrap first chunk ─────────────────────────────────────────────────
    with pi_lock:
        vel_cmd = pi_state['vel_cmd']

    pulses_c, last_dir, steps_c = compute_chunk_vel(
        vel_cmd, chunk_s, k,
        args.max_sps, args.deadband_sps,
        args.step, args.dir, last_dir, args.dir_invert,
        args.dir_blanking_us, args.dir_setup_us,
    )
    dur_c = wave_dur_s(pulses_c) or chunk_s

    pi_hw.wave_add_generic(pulses_c)
    wave_c       = pi_hw.wave_create()
    pi_hw.wave_send_once(wave_c)
    t_chunk_wall = time.monotonic()

    # ── Double-buffer main loop ───────────────────────────────────────────────
    #
    #   While chunk N plays via DMA, snapshot PI output and compute chunk N+1.
    #   Once N finishes: delete N (free CBs), add N+1, send N+1 immediately.
    #
    try:
        while True:
            if t_end_wall and time.monotonic() >= t_end_wall:
                break

            # Snapshot latest PI velocity command for next chunk
            with pi_lock:
                vel_cmd = pi_state['vel_cmd']

            pulses_n, dir_n, steps_n = compute_chunk_vel(
                vel_cmd, chunk_s, k,
                args.max_sps, args.deadband_sps,
                args.step, args.dir, last_dir, args.dir_invert,
                args.dir_blanking_us, args.dir_setup_us,
            )
            dur_n = wave_dur_s(pulses_n) or chunk_s

            # Sleep until ~5ms before chunk ends, then busy-poll
            sleep_s = dur_c - (time.monotonic() - t_chunk_wall) - 0.005
            if sleep_s > 0:
                time.sleep(sleep_s)
            while pi_hw.wave_tx_busy():
                time.sleep(0.0001)

            # Update position with exact step count from completed chunk
            s_est += steps_c
            x_est  = s_est / k

            if (not args.no_travel_safety) and abs(x_est) > HALF_TRAVEL_MM + 1.0:
                raise RuntimeError(
                    f"Travel safety trip: x_est={x_est:.2f}mm "
                    f"exceeds ±{HALF_TRAVEL_MM}mm"
                )

            # Delete current waveform before adding next (single CB pool occupancy)
            pi_hw.wave_delete(wave_c)
            pi_hw.wave_add_generic(pulses_n)
            wave_n = pi_hw.wave_create()
            pi_hw.wave_send_once(wave_n)
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

    except KeyboardInterrupt:
        pass

    finally:
        running[0] = False
        ctrl_t.join(timeout=0.5)
        stat_t.join(timeout=1.0)
        try:
            pi_hw.wave_tx_stop()
            pi_hw.wave_clear()
        except Exception:
            pass
        try:
            pi_hw.write(args.step, 0)
            pi_hw.write(args.dir,  0)
            pi_hw.write(ENABLE_PIN, 0)
        except Exception:
            pass

        print(f"\nStopped.  Final x_est = {s_est/k:+.2f}mm  ({int(s_est):+d} steps)")

        if not args.no_plot:
            print("Generating plot...")
            plot_run(log_buf, args)

# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    args = parse_args()

    if not args.simulate and not _HW_IMU_AVAILABLE:
        sys.exit(
            "ERROR: adafruit_lsm6ds not found.\n"
            "Install: pip install adafruit-circuitpython-lsm6ds\n"
            "Or run with --simulate."
        )

    k = k_steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    print("═" * 56)
    print("  VSS PI CONTROLLER  —  Velocity-Error Damping")
    print("═" * 56)
    print(f"  steps/mm  = {k:.4f}")
    print(f"  ±travel   = {HALF_TRAVEL_MM:.1f}mm  (≈{HALF_TRAVEL_MM * k:.0f} steps)")
    print(f"  simulate  = {args.simulate}")
    print(f"  Kp={args.kp}  Ki={args.ki}  τ_vel={args.tau_vel}s")
    print(f"  max_vel   = {args.max_vel_mm}mm/s  "
          f"chunk={args.chunk_ms}ms  control_fs={args.control_fs:.0f}Hz")

    pi_hw = pigpio.pi()
    if not pi_hw.connected:
        sys.exit("ERROR: pigpiod not running.  sudo systemctl start pigpiod")

    try:
        run_cancellation(pi_hw, args, k)
    finally:
        pi_hw.stop()

    print("Done.")


if __name__ == "__main__":
    main()

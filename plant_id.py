#!/usr/bin/env python3
"""
plant_id.py — VSS Secondary Path Identification
================================================
Runs the stepper open-loop in sinusoidal motion at each test frequency while
capturing IMU acceleration. Fits a sine to the IMU response and computes the
gain (dB) and phase (deg) of the path:

    stepper position (mm) → structural acceleration (m/s²)

This is the secondary path P that FxLMS needs to know.

Usage (on Pi, DC motor OFF, carriage near centre):
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 plant_id.py

Outputs in ./plant_id_out/:
  results.csv   — freq_hz, gain_db, phase_deg, amp_ms2, snr_db
  bode.png      — Bode magnitude + phase plot

Key flags:
  --amp_mm     Peak displacement amplitude (default 4 mm)
  --freqs      Space-separated list of test frequencies in Hz
  --settle     Cycles to discard per frequency (transient, default 4)
  --meas       Cycles to use for sine fitting (default 6)
  --imu_addr   LSM6DSO I2C address in hex (default 0x6b)
  --out_dir    Output directory (default ./plant_id_out)
  --baseline   Measure IMU noise floor before sweep (recommended)

Hardware assumed:
  STEP = BCM13 (PWM1), DIR = BCM16, EN = GPIO25 (active-low)
  LSM6DSO on I2C bus 1
  DM556 driver, 3200 steps/rev, STEPS_PER_MM ≈ 86
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pigpio
import smbus2

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    HAS_PLOT = True
except ImportError:
    HAS_PLOT = False
    print('[plant_id] matplotlib not found — CSV only, no plot')

# ── Hardware constants ──────────────────────────────────────────────────────────
STEP_PIN      = 13
DIR_PIN       = 16
EN_PIN        = 25
STEPS_PER_MM  = 86
MAX_AMP_MM    = 20.0    # hard ceiling on amplitude arg

# ── LSM6DSO registers ───────────────────────────────────────────────────────────
CTRL1_XL_REG  = 0x10
OUTX_L_A_REG  = 0x28   # x-axis accel low byte (high = +1)
# CTRL1_XL: ODR_XL=1000b (833 Hz), FS_XL=01b (±4 g) → 0x84
CTRL1_XL_VAL  = 0x84
ACCEL_SENS    = 0.122e-3 * 9.80665   # m/s² per LSB at ±4 g

# ── Default sweep parameters ────────────────────────────────────────────────────
DEFAULT_FREQS   = [5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
DEFAULT_AMP_MM  = 4.0
DEFAULT_SETTLE  = 4    # integer cycles to discard (avoids transient)
DEFAULT_MEAS    = 6    # integer cycles to fit (more = better SNR)
PWM_UPDATE_HZ   = 400  # velocity loop update rate (Hz)
DIR_SETUP_S     = 12e-6  # direction pin setup time before first step

# Global pigpio handle for clean shutdown
_pi = None


# ── IMU capture ─────────────────────────────────────────────────────────────────

class IMUCapture:
    """
    Reads LSM6DSO x-axis acceleration in a daemon thread at ~833 Hz.
    Timestamps via time.monotonic().
    """

    def __init__(self, bus_num=1, addr=0x6B):
        self.bus  = smbus2.SMBus(bus_num)
        self.addr = addr
        self._configure()
        self.lock     = threading.Lock()
        self._samples = []     # (t_monotonic, accel_ms2)
        self._running = False
        self._thread  = threading.Thread(target=self._loop, daemon=True)

    def _configure(self):
        self.bus.write_byte_data(self.addr, CTRL1_XL_REG, CTRL1_XL_VAL)
        time.sleep(0.02)

    def _read_x(self):
        lo  = self.bus.read_byte_data(self.addr, OUTX_L_A_REG)
        hi  = self.bus.read_byte_data(self.addr, OUTX_L_A_REG + 1)
        raw = np.frombuffer(bytes([lo, hi]), dtype=np.int16)[0]
        return float(raw) * ACCEL_SENS

    def _loop(self):
        dt   = 1.0 / 833.0
        next_t = time.monotonic()
        while self._running:
            t = time.monotonic()
            a = self._read_x()
            with self.lock:
                self._samples.append((t, a))
            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)

    def start(self):
        self._running = True
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join(timeout=2.0)

    def flush(self):
        """Discard buffered samples. Call at start of measurement window."""
        with self.lock:
            self._samples.clear()

    def get_samples(self):
        with self.lock:
            return list(self._samples)

    def noise_floor_rms(self, duration_s=2.0):
        """Measure RMS accel with no motion. Returns (rms, std)."""
        self.flush()
        time.sleep(duration_s)
        samps = self.get_samples()
        if not samps:
            return 0.0, 0.0
        a = np.array([s[1] for s in samps])
        return float(np.sqrt(np.mean(a**2))), float(np.std(a))


# ── Motion control ───────────────────────────────────────────────────────────────

def stepper_stop(pi):
    """Immediately halt stepper and disable driver."""
    pi.hardware_PWM(STEP_PIN, 0, 0)
    pi.write(EN_PIN, 1)   # active-low disable


def run_sine_motion(pi, imu, f, amplitude_mm, settle_cycles, meas_cycles):
    """
    Execute (settle + meas) complete cycles of sinusoidal stepper motion at
    frequency f. Returns (t_arr, a_arr) for the measurement window only,
    with t=0 at the measurement window start.

    The reference signal during the measurement window is:
        x_ref(τ) = amplitude_mm · sin(2π·f·τ),  τ ∈ [0, meas_time]
    (After integer settle_cycles the sine phase returns exactly to 0.)
    """
    omega      = 2.0 * np.pi * f
    T          = 1.0 / f
    settle_t   = settle_cycles * T
    meas_t     = meas_cycles   * T
    total_t    = settle_t + meas_t
    dt         = 1.0 / PWM_UPDATE_HZ
    flushed    = False

    pi.write(EN_PIN, 0)      # enable driver
    pi.write(DIR_PIN, 1)     # initial direction
    time.sleep(DIR_SETUP_S)

    prev_dir = 1
    t_abs_start = time.monotonic()
    t_now = 0.0

    while t_now < total_t:
        # Flush IMU exactly when measurement window begins
        if not flushed and t_now >= settle_t:
            imu.flush()
            flushed = True

        # Sinusoidal velocity: v(t) = A·ω·cos(ω·t)
        v_mms     = amplitude_mm * omega * np.cos(omega * t_now)
        v_steps_s = abs(v_mms) * STEPS_PER_MM   # steps/s ≥ 0

        # Direction change (with setup time)
        new_dir = 1 if v_mms >= 0.0 else 0
        if new_dir != prev_dir:
            pi.hardware_PWM(STEP_PIN, 0, 0)
            pi.write(DIR_PIN, new_dir)
            time.sleep(DIR_SETUP_S)
            prev_dir = new_dir

        # Set step rate via hardware PWM (50 % duty → clean pulses)
        step_hz = max(1, int(round(v_steps_s)))
        pi.hardware_PWM(STEP_PIN, step_hz, 500_000)

        # Sleep to next update tick
        t_now += dt
        t_target = t_abs_start + t_now
        sleep_s  = t_target - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)

    stepper_stop(pi)

    # ── Extract measurement window ──────────────────────────────────────────
    samps = imu.get_samples()
    if not samps:
        return None, None

    t0 = t_abs_start + settle_t            # absolute time at window start
    t_arr = np.array([s[0] for s in samps]) - t0
    a_arr = np.array([s[1] for s in samps])

    mask = (t_arr >= 0.0) & (t_arr <= meas_t)
    t_w  = t_arr[mask]
    a_w  = a_arr[mask]

    if len(t_w) < 10:
        return None, None

    return t_w, a_w


# ── Signal fitting ────────────────────────────────────────────────────────────

def fit_sine(t, y, f):
    """
    Least-squares fit of y(t) = C·cos(2πft) + S·sin(2πft) + bias.

    Phase convention (consistent with reference sin(2πft)):
      phase = arctan2(-S, C)
      → if y is a pure sin, C=0, S=A  → phase = arctan2(-A,0) = -π/2
      → if y is a pure cos, C=A, S=0  → phase = 0

    Returns: amplitude (m/s²), phase_rad, snr_db
    """
    X = np.column_stack([
        np.cos(2.0 * np.pi * f * t),
        np.sin(2.0 * np.pi * f * t),
        np.ones(len(t)),
    ])
    coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    C, S, bias = coeffs

    amplitude = np.hypot(C, S)
    phase     = np.arctan2(-S, C)

    y_fit   = X @ coeffs
    sig_pwr = float(np.var(y_fit - bias))
    res_pwr = float(np.var(y - y_fit))
    snr_db  = (10.0 * np.log10(sig_pwr / res_pwr)
               if res_pwr > 1e-20 else 99.0)

    return amplitude, phase, snr_db


def plant_tf(f, amp_in_mm, amp_out_ms2, phase_out_rad):
    """
    Compute plant transfer function from:
      input  = commanded stepper position (mm), reference x_ref = A·sin(2πft)
      output = IMU acceleration (m/s²)

    Reference phase: arctan2(-S_ref, C_ref) = arctan2(-A, 0) = -π/2
    Plant phase = phase_imu - phase_ref (wrapped to [-180, +180] deg)
    Plant gain  = |output amplitude (m/s²)| / |input amplitude (m)|
    """
    REF_PHASE = -np.pi / 2.0
    gain_lin  = amp_out_ms2 / (amp_in_mm * 1e-3)   # (m/s²)/m = s⁻²
    gain_db   = 20.0 * np.log10(max(gain_lin, 1e-12))
    phase_rad = np.angle(np.exp(1j * (phase_out_rad - REF_PHASE)))  # wrap [-π, π]
    phase_deg = np.degrees(phase_rad)
    return gain_db, phase_deg


# ── Bode plot ─────────────────────────────────────────────────────────────────

def plot_bode(results, noise_rms, out_dir):
    freqs  = [r['freq_hz']  for r in results]
    gains  = [r['gain_db']  for r in results]
    phases = [r['phase_deg'] for r in results]
    snrs   = [r['snr_db']   for r in results]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fig.suptitle('VSS Secondary Path — Bode Plot\n'
                 '(stepper position → structural acceleration)',
                 fontsize=12)

    sc1 = ax1.scatter(freqs, gains, c=snrs, cmap='RdYlGn',
                      vmin=0, vmax=25, s=90, zorder=5)
    ax1.plot(freqs, gains, '--', lw=1, alpha=0.5, color='gray')
    ax1.set_ylabel('Gain (dB)')
    ax1.grid(True, alpha=0.3)
    fig.colorbar(sc1, ax=ax1, label='SNR (dB)')

    # Mark noise floor on gain axis
    if noise_rms is not None:
        nf_db = 20.0 * np.log10(noise_rms / (DEFAULT_AMP_MM * 1e-3) + 1e-12)
        ax1.axhline(nf_db, color='red', linestyle=':', lw=1, alpha=0.7,
                    label=f'noise floor (~{nf_db:.0f} dB)')
        ax1.legend(fontsize=9)

    sc2 = ax2.scatter(freqs, phases, c=snrs, cmap='RdYlGn',
                      vmin=0, vmax=25, s=90, zorder=5)
    ax2.plot(freqs, phases, '--', lw=1, alpha=0.5, color='gray')
    ax2.axhline(  0, color='green',  linestyle=':', lw=1, alpha=0.7, label='0° (pure gain)')
    ax2.axhline(-90, color='orange', linestyle=':', lw=1, alpha=0.7, label='−90° (integrator)')
    ax2.set_ylabel('Phase (deg)')
    ax2.set_xlabel('Frequency (Hz)')
    ax2.set_ylim(-200, 200)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    fig.colorbar(sc2, ax=ax2, label='SNR (dB)')

    plt.tight_layout()
    out = Path(out_dir) / 'bode.png'
    plt.savefig(out, dpi=150)
    plt.close()
    print(f'[plant_id] Bode plot → {out}')


# ── Main ───────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description='VSS open-loop plant identification sweep')
    p.add_argument('--freqs', nargs='+', type=float, default=DEFAULT_FREQS,
                   metavar='F', help='Test frequencies in Hz')
    p.add_argument('--amp_mm', type=float, default=DEFAULT_AMP_MM,
                   help='Peak stepper displacement (mm, default 4)')
    p.add_argument('--settle', type=int, default=DEFAULT_SETTLE,
                   help='Settling cycles per frequency (discarded, default 4)')
    p.add_argument('--meas', type=int, default=DEFAULT_MEAS,
                   help='Measurement cycles for sine fit (default 6)')
    p.add_argument('--imu_addr', type=lambda x: int(x, 0), default=0x6B,
                   help='LSM6DSO I2C address (default 0x6b)')
    p.add_argument('--out_dir', type=str, default='./plant_id_out')
    p.add_argument('--baseline', action='store_true',
                   help='Measure IMU noise floor before sweep (recommended)')
    p.add_argument('--no_return', action='store_true',
                   help='Skip slow centre-return between frequencies')
    return p.parse_args()


def main():
    global _pi
    args = parse_args()

    if args.amp_mm > MAX_AMP_MM:
        sys.exit(f'[plant_id] --amp_mm {args.amp_mm} exceeds safety ceiling '
                 f'{MAX_AMP_MM} mm — aborting')

    os.makedirs(args.out_dir, exist_ok=True)

    # ── pigpio setup ──────────────────────────────────────────────────────────
    _pi = pigpio.pi()
    if not _pi.connected:
        sys.exit('[plant_id] pigpiod not running. Start with: sudo pigpiod -s 2')

    _pi.set_mode(DIR_PIN, pigpio.OUTPUT)
    _pi.set_mode(EN_PIN,  pigpio.OUTPUT)
    stepper_stop(_pi)    # ensure disabled at start

    def _shutdown(sig=None, frame=None):
        print('\n[plant_id] Interrupted — stopping stepper')
        stepper_stop(_pi)
        _pi.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── IMU setup ─────────────────────────────────────────────────────────────
    try:
        imu = IMUCapture(addr=args.imu_addr)
    except Exception as e:
        stepper_stop(_pi)
        _pi.stop()
        sys.exit(f'[plant_id] IMU init failed: {e}')

    imu.start()
    print('[plant_id] IMU started. Warming up 1 s...')
    time.sleep(1.0)

    # ── Baseline noise floor ──────────────────────────────────────────────────
    noise_rms = None
    if args.baseline:
        print('[plant_id] Measuring IMU noise floor (2 s, motor OFF)...')
        noise_rms, noise_std = imu.noise_floor_rms(2.0)
        print(f'  Noise RMS = {noise_rms*1e3:.3f} mm/s²  std = {noise_std*1e3:.3f} mm/s²')

    # ── Frequency sweep ───────────────────────────────────────────────────────
    results = []
    freqs_sorted = sorted(set(args.freqs))

    for f in freqs_sorted:
        n_total = args.settle + args.meas
        print(f'\n[plant_id] ── {f:.1f} Hz  amp={args.amp_mm} mm  '
              f'cycles={n_total} ({args.settle}settle+{args.meas}meas)')

        try:
            t_w, a_w = run_sine_motion(
                _pi, imu, f,
                amplitude_mm=args.amp_mm,
                settle_cycles=args.settle,
                meas_cycles=args.meas,
            )
        except Exception as e:
            print(f'  ERROR during motion: {e}')
            stepper_stop(_pi)
            continue

        if t_w is None or len(t_w) < 20:
            print('  WARNING: too few IMU samples — skipping this frequency')
            continue

        amp_imu, phase_imu, snr = fit_sine(t_w, a_w, f)
        gain_db, phase_deg      = plant_tf(f, args.amp_mm, amp_imu, phase_imu)

        print(f'  IMU amp  = {amp_imu*1e3:.2f} mm/s²')
        print(f'  Gain     = {gain_db:.1f} dB')
        print(f'  Phase    = {phase_deg:.1f} °')
        print(f'  SNR      = {snr:.1f} dB  (samples={len(t_w)})')

        if snr < 6.0:
            print('  ⚠  Low SNR — consider increasing --amp_mm or --meas')

        results.append({
            'freq_hz':   f,
            'gain_db':   round(gain_db,  2),
            'phase_deg': round(phase_deg, 1),
            'amp_ms2':   round(amp_imu,  6),
            'snr_db':    round(snr,       1),
        })

        # Brief pause between frequencies (let carriage settle near zero)
        if not args.no_return:
            time.sleep(0.5)

    # ── Stop IMU + pigpio ────────────────────────────────────────────────────
    imu.stop()
    stepper_stop(_pi)
    _pi.stop()

    if not results:
        sys.exit('[plant_id] No results — check hardware and try again')

    # ── Save CSV ─────────────────────────────────────────────────────────────
    csv_path = Path(args.out_dir) / 'results.csv'
    with open(csv_path, 'w', newline='') as fh:
        w = csv.DictWriter(
            fh, fieldnames=['freq_hz','gain_db','phase_deg','amp_ms2','snr_db'])
        w.writeheader()
        w.writerows(results)
    print(f'\n[plant_id] CSV → {csv_path}')

    # ── Print table ───────────────────────────────────────────────────────────
    print(f'\n{"Freq":>6}  {"Gain":>8}  {"Phase":>9}  {"SNR":>7}  {"IMU amp":>12}')
    print('─' * 52)
    for r in results:
        flag = '  ⚠ low SNR' if r['snr_db'] < 6 else ''
        print(f"{r['freq_hz']:>6.1f}  {r['gain_db']:>7.1f}dB  "
              f"{r['phase_deg']:>8.1f}°  {r['snr_db']:>6.1f}dB  "
              f"{r['amp_ms2']*1e3:>9.2f} mm/s²{flag}")

    # ── Bode plot ─────────────────────────────────────────────────────────────
    if HAS_PLOT:
        plot_bode(results, noise_rms, args.out_dir)

    # ── Key interpretation note ───────────────────────────────────────────────
    phases = [r['phase_deg'] for r in results]
    if phases:
        mean_phase = float(np.mean(phases))
        print(f'\n[plant_id] Mean secondary path phase across sweep: {mean_phase:.1f}°')
        if abs(mean_phase) < 30:
            print('  → Phase near 0° — standard LMS may converge but FxLMS still safer')
        elif abs(mean_phase) < 90:
            print('  → Significant phase — FxLMS correction required')
        else:
            print('  → Phase > 90° — FxLMS essential; standard LMS will diverge')


if __name__ == '__main__':
    main()

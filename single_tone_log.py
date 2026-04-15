#!/usr/bin/env python3
"""
single_tone_log.py  —  Open-loop single-frequency drive + IMU logger
─────────────────────────────────────────────────────────────────────
Drives the stepper in pure sinusoidal motion at a fixed frequency and
amplitude while simultaneously logging IMU x-axis acceleration at ~800 Hz.
Saves a CSV that plot_plant_phase.py uses to extract the true plant phase φ_P.

The plant secondary path is:
    stepper position [mm]  →  base-plate acceleration [m/s²]
Comparing the commanded position sine to the measured acceleration sine
gives the true φ_P (including structural dynamics, not just a kinematic
ω² factor).  This is exactly the phase FxLMS needs in its secondary-path
model.

USAGE
─────
  # 1. Confirm DC motor is OFF and mechanically stopped.
  # 2. Run (must be root for pigpio real-time priority):
  sudo chrt -f 50 /home/vibess/vss-venv/bin/python3 single_tone_log.py \\
      --amp_mm 5 --freq 8.5 --settle 5 --log_dur 15

  Outputs: single_tone_out/log_<timestamp>.csv

HARDWARE
────────
  STEP  BCM 13 (PWM1), DIR BCM 16, ENA BCM 25 (active-low on DM556)
  IMU   LSM6DSO32 at I2C 0x6A, bus 1
  pigpiod must be running: sudo pigpiod -s 2 -b 4096
"""

import argparse
import math
import os
import sys
import time
import threading
import csv
from datetime import datetime

import numpy as np
import pigpio

try:
    from smbus2 import SMBus
except ImportError:
    print("ERROR: smbus2 not installed.  Run: pip install smbus2")
    sys.exit(1)

# ═══════════════════════════════════════════════════════════════════════════
# Hardware constants  (match waveStepper.py / vss_controller.py exactly)
# ═══════════════════════════════════════════════════════════════════════════

STEP_PIN        = 13
DIR_PIN         = 16
ENA_PIN         = 25
PULSE_HIGH_US   = 5           # DM556 needs ≥ 5 µs high pulse

STEPS_PER_MM    = 86.0297     # 3200 steps/rev, pulley ⌀ 11.84 mm
HALF_TRAVEL_MM  = 75.0        # physical end-to-end limit

CHUNK_DUR_S     = 0.050       # 50 ms DMA chunks — same as waveStepper.py

I2C_BUS         = 1
IMU_ADDR        = 0x6A
CTRL1_XL        = 0x10
OUTX_L_A        = 0x28
# ODR = 833 Hz (bits [7:4] = 0111), FS = ±32 g (bits [3:2] = 11)
CTRL1_XL_VAL    = 0x7C
ACCEL_SCALE     = 32.0 * 9.80665 / 32768.0   # m/s² per LSB at ±32 g

OUT_DIR         = "single_tone_out"

# ═══════════════════════════════════════════════════════════════════════════
# IMU helpers  (direct smbus2 — no adafruit dependency)
# ═══════════════════════════════════════════════════════════════════════════

def imu_init(bus: SMBus) -> None:
    bus.write_byte_data(IMU_ADDR, CTRL1_XL, CTRL1_XL_VAL)
    time.sleep(0.05)  # wait for ODR to settle

def imu_read_x(bus: SMBus) -> float:
    """Return x-axis acceleration in m/s² (little-endian two's complement)."""
    data = bus.read_i2c_block_data(IMU_ADDR, OUTX_L_A, 2)
    raw  = (data[1] << 8) | data[0]
    if raw & 0x8000:
        raw -= 0x10000
    return raw * ACCEL_SCALE

# ═══════════════════════════════════════════════════════════════════════════
# IMU logging thread
# ═══════════════════════════════════════════════════════════════════════════

class ImuLogger:
    """
    Runs in its own thread.  Busy-wait loop targeting ~800 Hz.
    Stores (timestamp, accel_x) pairs in a pre-allocated buffer.

    Why busy-wait?  time.sleep() on Linux has ~1 ms jitter — at 800 Hz
    a sample period is 1.25 ms, so sleep-based timing would miss samples
    unpredictably.  Busy-waiting burns a core but keeps jitter < 50 µs.
    """

    TARGET_FS = 800  # Hz

    def __init__(self, duration_s: float):
        n = int(duration_s * self.TARGET_FS * 1.5)  # 50% headroom
        self.t_buf   = np.zeros(n, dtype=np.float64)
        self.a_buf   = np.zeros(n, dtype=np.float64)
        self.count   = 0
        self._stop   = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join()

    def _run(self):
        bus = SMBus(I2C_BUS)
        imu_init(bus)
        period = 1.0 / self.TARGET_FS
        next_t = time.perf_counter()
        try:
            while not self._stop.is_set():
                while time.perf_counter() < next_t:
                    pass  # busy-wait for next slot
                t_now = time.perf_counter()
                try:
                    ax = imu_read_x(bus)
                except OSError:
                    ax = self.a_buf[self.count - 1] if self.count > 0 else 0.0
                if self.count < len(self.t_buf):
                    self.t_buf[self.count] = t_now
                    self.a_buf[self.count] = ax
                    self.count += 1
                next_t += period
        finally:
            bus.close()

    def get_data(self):
        """Return (t_array, accel_array) trimmed to actual sample count."""
        n = self.count
        return self.t_buf[:n].copy(), self.a_buf[:n].copy()

# ═══════════════════════════════════════════════════════════════════════════
# Stepper waveform chunk generation
# ═══════════════════════════════════════════════════════════════════════════

def build_chunk(pi: pigpio.pi,
                t_chunk_start: float,
                amp_mm: float,
                omega: float,
                chunk_dur: float,
                max_sps: int) -> tuple:
    """
    Build one DMA waveform chunk for x(t) = amp_mm * sin(omega * t).

    Strategy: sample the desired position at fine sub-steps within the chunk,
    convert to step counts, and emit pulses at the inter-step timing implied
    by the velocity at each point.

    Returns (wave_id, n_steps_total) or raises RuntimeError on overflow.

    Why sinusoidal, not trapezoidal?
    ─────────────────────────────────
    A trapezoidal profile has velocity discontinuities — the plant sees a
    square-wave force input, which excites many harmonics and makes phase
    measurement ambiguous.  A sinusoidal profile is a single-frequency input;
    the IMU response is also (approximately) single-frequency, so a sine fit
    extracts a clean φ_P.
    """

    # Sub-sample the chunk at 10× the max step rate for accurate timing
    sub_steps = max(int(chunk_dur * max_sps * 10), 1000)
    t_fine   = np.linspace(t_chunk_start, t_chunk_start + chunk_dur, sub_steps + 1)
    pos_fine = amp_mm * np.sin(omega * t_fine)        # mm
    step_fine = pos_fine * STEPS_PER_MM               # fractional steps

    pulses = []
    pos_prev = step_fine[0]

    for i in range(1, len(t_fine)):
        pos_curr = step_fine[i]
        delta    = pos_curr - pos_prev
        if abs(delta) < 1.0:
            continue

        n_steps  = int(abs(delta))
        if n_steps == 0:
            continue

        direction = 1 if delta > 0 else 0
        dt_each_us = max(int((t_fine[i] - t_fine[i - 1]) * 1e6 / n_steps), PULSE_HIGH_US * 2 + 1)

        # Clamp to max_sps
        min_dt_us = int(1e6 / max_sps)
        dt_each_us = max(dt_each_us, min_dt_us)

        # Set direction
        dir_pulse = pigpio.pulse(1 << DIR_PIN if direction else 0,
                                 0 if direction else 1 << DIR_PIN,
                                 1)
        pulses.append(dir_pulse)

        for _ in range(n_steps):
            pulses.append(pigpio.pulse(1 << STEP_PIN, 0,             PULSE_HIGH_US))
            pulses.append(pigpio.pulse(0,             1 << STEP_PIN, dt_each_us - PULSE_HIGH_US))

        pos_prev += n_steps * (1 if direction else -1)

    if not pulses:
        # Dead chunk (e.g. exactly at a zero crossing with no motion)
        # Add a 1 µs no-op so pigpio doesn't complain about an empty wave
        pulses.append(pigpio.pulse(0, 0, int(chunk_dur * 1e6)))

    pi.wave_add_generic(pulses)
    wave_id = pi.wave_create()
    if wave_id < 0:
        raise RuntimeError(f"wave_create() returned {wave_id} — DMA resource error")
    return wave_id

# ═══════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════

def parse_args():
    p = argparse.ArgumentParser(description="Single-tone open-loop drive + IMU log")
    p.add_argument("--amp_mm",  type=float, default=5.0,
                   help="Sinusoidal amplitude in mm (default 5)")
    p.add_argument("--freq",    type=float, default=8.5,
                   help="Drive frequency in Hz (default 8.5)")
    p.add_argument("--settle",  type=float, default=5.0,
                   help="Seconds to run before logging starts (default 5)")
    p.add_argument("--log_dur", type=float, default=15.0,
                   help="Seconds of IMU data to capture (default 15)")
    p.add_argument("--max_sps", type=int,   default=16000,
                   help="Maximum step rate in steps/s (default 16000)")
    return p.parse_args()


def main():
    args = parse_args()

    omega      = 2.0 * math.pi * args.freq
    total_dur  = args.settle + args.log_dur
    amp_steps  = args.amp_mm * STEPS_PER_MM

    # ── Sanity checks ─────────────────────────────────────────────────────
    if args.amp_mm > HALF_TRAVEL_MM:
        sys.exit(f"ERROR: amp_mm={args.amp_mm} exceeds half-travel {HALF_TRAVEL_MM} mm")

    peak_sps = args.amp_mm * omega * STEPS_PER_MM
    if peak_sps > args.max_sps:
        sys.exit(f"ERROR: Peak step rate {peak_sps:.0f} sps exceeds --max_sps {args.max_sps}.\n"
                 f"       Reduce --amp_mm or --freq, or increase --max_sps.")

    print(f"┌─ single_tone_log ─────────────────────────────────────────┐")
    print(f"│  freq     = {args.freq} Hz")
    print(f"│  amp      = {args.amp_mm} mm  ({amp_steps:.0f} steps peak)")
    print(f"│  peak sps = {peak_sps:.0f}  (limit {args.max_sps})")
    print(f"│  settle   = {args.settle} s   log = {args.log_dur} s")
    print(f"│  total    = {total_dur} s")
    print(f"└───────────────────────────────────────────────────────────┘")
    print()

    # ── pigpio init ────────────────────────────────────────────────────────
    pi = pigpio.pi()
    if not pi.connected:
        sys.exit("ERROR: Cannot connect to pigpiod.  Run: sudo pigpiod -s 2 -b 4096")

    pi.set_mode(STEP_PIN, pigpio.OUTPUT)
    pi.set_mode(DIR_PIN,  pigpio.OUTPUT)
    pi.set_mode(ENA_PIN,  pigpio.OUTPUT)

    # ENA is active-LOW on DM556 — pull low to enable driver
    pi.write(ENA_PIN, 0)
    pi.write(STEP_PIN, 0)
    time.sleep(0.1)

    # ── Start IMU logger ───────────────────────────────────────────────────
    logger = ImuLogger(duration_s=total_dur + 2)
    logger.start()

    t_global_start = time.perf_counter()
    print(f"t=0.00  Drive started.  Settling for {args.settle} s ...")

    # ── Double-buffer waveform loop ────────────────────────────────────────
    # Why double-buffer?  A single-buffer approach would have a gap between
    # waves while Python computes the next one (~1–2 ms).  At 8.5 Hz that's
    # a visible glitch in the position profile.  Double-buffering lets us
    # pre-compute the next chunk while the current one plays, eliminating gaps.

    t_chunk = 0.0   # logical time (seconds from motion start)

    try:
        # Pre-build first two chunks before starting playback
        wave_id_cur  = build_chunk(pi, t_chunk,                   args.amp_mm, omega, CHUNK_DUR_S, args.max_sps)
        t_chunk     += CHUNK_DUR_S
        wave_id_next = build_chunk(pi, t_chunk,                   args.amp_mm, omega, CHUNK_DUR_S, args.max_sps)

        pi.wave_send_once(wave_id_cur)
        t_wave_fire = time.perf_counter()   # when the first pulse actually fires
        wave_id_old = wave_id_cur

        while True:
            elapsed = time.perf_counter() - t_global_start
            if elapsed >= total_dur:
                break

            # Wait for current wave to finish
            while pi.wave_tx_busy():
                time.sleep(0.001)

            # Fire next, compute the one after
            pi.wave_send_once(wave_id_next)
            t_chunk += CHUNK_DUR_S

            pi.wave_delete(wave_id_old)
            wave_id_old  = wave_id_next
            wave_id_next = build_chunk(pi, t_chunk, args.amp_mm, omega, CHUNK_DUR_S, args.max_sps)

            # Status print
            remaining = total_dur - elapsed
            phase_str = "SETTLING" if elapsed < args.settle else "LOGGING "
            print(f"  t={elapsed:6.2f}s  [{phase_str}]  {remaining:.1f}s remaining", end="\r")

        print()

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        pi.wave_tx_stop()
        try:
            pi.wave_delete(wave_id_old)
            pi.wave_delete(wave_id_next)
        except Exception:
            pass
        # Return carriage to centre — send it to x=0
        pi.write(STEP_PIN, 0)
        pi.write(ENA_PIN, 1)   # disable driver
        pi.stop()

    logger.stop()

    # ── Trim log to the logging window only ────────────────────────────────
    t_raw, a_raw = logger.get_data()
    if len(t_raw) == 0:
        sys.exit("ERROR: No IMU samples captured.")

    # Convert absolute timestamps → time-since-wave-fire
    t_rel = t_raw - t_wave_fire

    # Keep only the logging window (after settle, before total_dur)
    mask = (t_rel >= args.settle) & (t_rel <= total_dur)
    t_log = t_rel[mask]
    a_log = a_raw[mask]

    actual_fs = len(t_log) / (t_log[-1] - t_log[0]) if len(t_log) > 1 else 0
    print(f"\nCaptured {len(t_log)} IMU samples over {t_log[-1]-t_log[0]:.2f} s")
    print(f"Actual IMU sample rate: {actual_fs:.1f} Hz")

    # ── Save CSV ───────────────────────────────────────────────────────────
    os.makedirs(OUT_DIR, exist_ok=True)
    ts_str   = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = os.path.join(OUT_DIR, f"log_{ts_str}.csv")

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        # Header carries the run parameters so the plot script is self-contained
        writer.writerow(["# freq_hz",    args.freq])
        writer.writerow(["# amp_mm",     args.amp_mm])
        writer.writerow(["# settle_s",   args.settle])
        writer.writerow(["# log_dur_s",  args.log_dur])
        writer.writerow(["# steps_per_mm", STEPS_PER_MM])
        writer.writerow(["t_s", "accel_x_mps2"])
        for t, a in zip(t_log, a_log):
            writer.writerow([f"{t:.6f}", f"{a:.6f}"])

    print(f"Saved → {out_path}")
    print(f"\nRun:  python3 plot_plant_phase.py {out_path}")

if __name__ == "__main__":
    main()

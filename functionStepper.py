#!/usr/bin/env python3
import argparse
import math
import time
import pigpio

# Mechanics defaults
DEFAULT_PULLEY_D_MM = 11.84
DEFAULT_PULSES_PER_REV = 800

TRAVEL_TOTAL_MM = 150.0
HALF_TRAVEL_MM = TRAVEL_TOTAL_MM / 2.0

DEFAULT_STEP_BCM = 18   # physical pin 12 (PWM0)
DEFAULT_DIR_BCM = 16

def steps_per_mm(pulley_d_mm: float, pulses_per_rev: float) -> float:
    return pulses_per_rev / (math.pi * pulley_d_mm)

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def parse_args():
    p = argparse.ArgumentParser("VSS function stepper (hardware PWM STEP)")
    p.add_argument("--step", type=int, default=DEFAULT_STEP_BCM, help="BCM STEP pin (use 18 for hardware PWM)")
    p.add_argument("--dir", type=int, default=DEFAULT_DIR_BCM, help="BCM DIR pin")
    p.add_argument("--dir_invert", action="store_true", help="Invert DIR polarity")

    p.add_argument("--pulley_d_mm", type=float, default=DEFAULT_PULLEY_D_MM, help="Pulley diameter (mm)")
    p.add_argument("--pulses_per_rev", type=float, default=DEFAULT_PULSES_PER_REV, help="Driver pulses per motor rev")

    # Up to 3 tones: x(t) = sum Ai * sin(2πfi t + phii)
    p.add_argument("--a1", type=float, default=0.0, help="Amplitude 1 (mm, peak)")
    p.add_argument("--f1", type=float, default=0.0, help="Frequency 1 (Hz)")
    p.add_argument("--a2", type=float, default=0.0, help="Amplitude 2 (mm, peak)")
    p.add_argument("--f2", type=float, default=0.0, help="Frequency 2 (Hz)")
    p.add_argument("--a3", type=float, default=0.0, help="Amplitude 3 (mm, peak)")
    p.add_argument("--f3", type=float, default=0.0, help="Frequency 3 (Hz)")

    p.add_argument("--phi1", type=float, default=0.0, help="Phase 1 (deg)")
    p.add_argument("--phi2", type=float, default=0.0, help="Phase 2 (deg)")
    p.add_argument("--phi3", type=float, default=0.0, help="Phase 3 (deg)")

    p.add_argument("--rate", type=float, default=200.0, help="Update rate (Hz), 50–400 typical")
    p.add_argument("--dur", type=float, default=0.0, help="Duration seconds, 0=run forever")

    p.add_argument("--max_sps", type=float, default=8000.0, help="Max steps/sec clamp")
    p.add_argument("--deadband_sps", type=float, default=0.0, help="Below this, set PWM off (prevents chatter)")
    p.add_argument("--slew_sps_per_s", type=float, default=50000.0, help="Max step-rate change per second")

    p.add_argument("--dir_setup_us", type=int, default=20, help="DIR setup time before stepping (us)")
    p.add_argument("--dir_blanking_us", type=int, default=200, help="STEP-off blanking around DIR flips (us)")

    p.add_argument("--duty", type=int, default=500000, help="PWM duty (0..1_000_000). 500000=50%")
    p.add_argument("--status_hz", type=float, default=2.0, help="Print status at this rate (Hz), 0=off")

    p.add_argument("--const_sps", type=float, default=0.0, help="If >0, run constant step rate (steps/sec)")
    p.add_argument("--const_dir", type=int, default=1, help="Direction for const_sps mode (0 or 1)")

    p.add_argument("--no_travel_safety", action="store_true", help="Disable travel safety bound (use carefully)")

    return p.parse_args()

def main():
    args = parse_args()

    if args.step != 18:
        print("WARNING: STEP is not BCM18. hardware_PWM is best on 18/12/13/19; BCM18 recommended.")

    if not (0 <= args.duty <= 1_000_000):
        raise SystemExit("ERROR: --duty must be 0..1_000_000")

    k = steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    A = [args.a1, args.a2, args.a3]
    F = [args.f1, args.f2, args.f3]
    PHI = [math.radians(args.phi1), math.radians(args.phi2), math.radians(args.phi3)]

    # Travel safety: worst-case displacement is sum |Ai|
    if (not args.no_travel_safety) and (not const_mode):
        if abs(x_est) > HALF_TRAVEL_MM + 1.0:
            raise RuntimeError(
                f"Travel safety trip: estimated x={x_est:.2f} mm exceeds ±{HALF_TRAVEL_MM:.2f} mm."
            )

    # Conservative peak velocity bound: vmax <= Σ |Ai| * 2πfi
    vmax_mm_s = 0.0
    for i in range(3):
        if A[i] != 0.0 and F[i] != 0.0:
            vmax_mm_s += abs(A[i]) * (2.0 * math.pi * abs(F[i]))
    est_peak_sps = vmax_mm_s * k
    if est_peak_sps > args.max_sps + 1e-9:
        raise SystemExit(
            f"ERROR: estimated peak step rate ~{est_peak_sps:.0f} sps exceeds max_sps={args.max_sps:.0f}.\n"
            f"Lower amplitudes/frequencies or raise max_sps only after testing."
        )

    fs = float(args.rate)
    if fs <= 0:
        raise SystemExit("ERROR: --rate must be > 0")
    dt = 1.0 / fs

    const_mode = args.const_sps > 0.0
    const_freq = int(round(clamp(args.const_sps, 0.0, args.max_sps)))
    const_dir = 1 if args.const_dir else 0
    if args.dir_invert:
        const_dir ^= 1

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("ERROR: pigpio not connected. Is pigpiod running?")

    pi.set_mode(args.dir, pigpio.OUTPUT)
    pi.write(args.dir, 0)

    # Start with STEP off
    pi.hardware_PWM(args.step, 0, 0)

    dir_invert = 1 if args.dir_invert else 0
    last_dir = None

    # Estimated position for safety (integrate what we command)
    s_est = 0.0
    sps_prev = 0.0

    # Cache last PWM command to reduce pigpio churn
    last_pwm_freq = None
    last_pwm_duty = None
    pwm_is_on = False

    # Use monotonic time to avoid NTP/system time jumps causing multi-second sleeps
    t0 = time.monotonic()
    deadline = t0
    t_end = (t0 + args.dur) if args.dur > 0 else None

    # Status printing scheduler
    status_period = (1.0 / args.status_hz) if args.status_hz and args.status_hz > 0 else None
    next_status = t0 if status_period else None

    print(f"steps_per_mm = {k:.4f}")
    print(f"half_travel = {HALF_TRAVEL_MM:.1f} mm (~{HALF_TRAVEL_MM*k:.0f} steps)")
    print(f"est peak sps ~ {est_peak_sps:.0f} (limit {args.max_sps:.0f})")
    print(f"update rate = {fs:.1f} Hz (dt={dt*1000:.1f} ms)")
    print("Ctrl-C to stop.")

    def pwm_off():
        nonlocal last_pwm_freq, last_pwm_duty, pwm_is_on
        if pwm_is_on:
            pi.hardware_PWM(args.step, 0, 0)
            pwm_is_on = False
            last_pwm_freq = 0
            last_pwm_duty = 0

    def pwm_set(freq_hz: int, duty: int):
        nonlocal last_pwm_freq, last_pwm_duty, pwm_is_on
        if freq_hz <= 0:
            pwm_off()
            return
        if (last_pwm_freq != freq_hz) or (last_pwm_duty != duty) or (not pwm_is_on):
            pi.hardware_PWM(args.step, freq_hz, duty)
            pwm_is_on = True
            last_pwm_freq = freq_hz
            last_pwm_duty = duty

    try:
        while True:
            now = time.monotonic()
            if t_end is not None and now >= t_end:
                break

            t = now - t0

            if const_mode:
                v_mm_s = 0.0  # not used
                sps_cmd = float(const_freq)
                logical_dir = const_dir
            else:
                # Desired velocity v(t) = d/dt Σ Ai sin(2πfi t + phii)
                v_mm_s = 0.0
                for i in range(3):
                    if A[i] == 0.0 or F[i] == 0.0:
                        continue
                    w = 2.0 * math.pi * F[i]
                    v_mm_s += A[i] * w * math.cos(w * t + PHI[i])

                # Direction from velocity sign
                logical_dir = 1 if v_mm_s >= 0.0 else 0
                logical_dir ^= dir_invert

                # Command step rate magnitude
                sps_cmd = abs(v_mm_s) * k
                if sps_cmd < args.deadband_sps:
                    sps_cmd = 0.0
                sps_cmd = clamp(sps_cmd, 0.0, args.max_sps)

            # Slew limit (prevents sudden sps jumps) — applies to both modes
            dsps_max = max(0.0, args.slew_sps_per_s) * dt
            sps = clamp(sps_cmd, sps_prev - dsps_max, sps_prev + dsps_max)
            sps_prev = sps

            # Handle DIR flips without inserting a full dt pause
            if last_dir is None:
                pi.write(args.dir, logical_dir)
                time.sleep(args.dir_setup_us / 1e6)
                last_dir = logical_dir
            elif logical_dir != last_dir:
                pwm_off()
                # short blanking only; DO NOT sleep(dt)
                if args.dir_blanking_us > 0:
                    time.sleep(args.dir_blanking_us / 1e6)
                pi.write(args.dir, logical_dir)
                time.sleep(args.dir_setup_us / 1e6)
                last_dir = logical_dir

            # Apply PWM
            if sps <= 0.0:
                pwm_off()
                steps_delivered = 0.0
                freq = 0
            else:
                freq = int(round(sps))
                if freq < 1:
                    pwm_off()
                    steps_delivered = 0.0
                else:
                    pwm_set(freq, args.duty)
                    steps_delivered = float(freq) * dt  # approx

            # Update estimated position for safety bound
            phys_dir = (logical_dir ^ dir_invert)
            phys_dir = (logical_dir ^ dir_invert)
            sign = 1.0 if phys_dir == 1 else -1.0
            s_est += sign * steps_delivered
            x_est = s_est / k
            if abs(x_est) > HALF_TRAVEL_MM + 1.0:
                raise RuntimeError(
                    f"Travel safety trip: estimated x={x_est:.2f} mm exceeds ±{HALF_TRAVEL_MM:.2f} mm."
                )

            # Status print
            if next_status is not None and now >= next_status:
                print(f"t={t:7.3f}s  sps={freq:6d}  v={v_mm_s:+8.2f} mm/s  x_est={x_est:+7.2f} mm  dir={last_dir}  pwm_on={int(pwm_is_on)} last_pwm={last_pwm_freq}")
                next_status += status_period

            # Deadline-based pacing (prevents long random sleeps)
            deadline += dt
            sleep_s = deadline - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # if we fell behind, snap deadline to now to avoid spiral
                deadline = time.monotonic()

    except KeyboardInterrupt:
        pass
    finally:
        try:
            pi.hardware_PWM(args.step, 0, 0)
            pi.write(args.dir, 0)
        finally:
            pi.stop()
        print("stopped")

if __name__ == "__main__":
    main()

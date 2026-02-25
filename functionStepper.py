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

DEFAULT_STEP_BCM = 18   # physical pin 12
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

    # Control update rate: how often we recompute the desired PWM frequency
    p.add_argument("--rate", type=float, default=200.0, help="Update rate (Hz), 50–400 typical")
    p.add_argument("--dur", type=float, default=0.0, help="Duration seconds, 0=run forever")

    p.add_argument("--max_sps", type=float, default=8000.0, help="Max steps/sec clamp")
    p.add_argument("--deadband_sps", type=float, default=5.0, help="Below this, set PWM off (prevents chatter)")
    p.add_argument("--slew_sps_per_s", type=float, default=50000.0, help="Max step-rate change per second")

    p.add_argument("--dir_setup_us", type=int, default=20, help="DIR setup time before stepping (us)")
    p.add_argument("--duty", type=int, default=500000, help="PWM duty (0..1_000_000). 500000=50%")

    return p.parse_args()


def main():
    args = parse_args()

    if args.step != 18:
        print("WARNING: --step is not BCM 18. hardware_PWM works best on BCM 18/12/13/19; BCM18 is recommended.")

    if not (0 <= args.duty <= 1_000_000):
        raise SystemExit("ERROR: --duty must be 0..1_000_000")

    k = steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    A = [args.a1, args.a2, args.a3]
    F = [args.f1, args.f2, args.f3]
    PHI = [math.radians(args.phi1), math.radians(args.phi2), math.radians(args.phi3)]

    # Travel safety: worst-case displacement is sum |Ai|
    sumA = sum(abs(x) for x in A)
    if sumA > HALF_TRAVEL_MM + 1e-9:
        raise SystemExit(
            f"ERROR: sum(|Ai|)={sumA:.3f} mm exceeds half-travel {HALF_TRAVEL_MM:.3f} mm."
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

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("ERROR: pigpio not connected. Is pigpiod running?")

    pi.set_mode(args.dir, pigpio.OUTPUT)
    pi.write(args.dir, 0)

    # Start with STEP off
    pi.hardware_PWM(args.step, 0, 0)

    dir_invert = 1 if args.dir_invert else 0
    last_dir = None

    # Estimated position (for safety), integrate delivered steps ≈ sps * dt
    s_est = 0.0  # steps
    sps_prev = 0.0

    t0 = time.time()
    t_end = (t0 + args.dur) if args.dur > 0 else None

    print(f"steps_per_mm = {k:.4f}")
    print(f"half_travel  = {HALF_TRAVEL_MM:.1f} mm (~{HALF_TRAVEL_MM*k:.0f} steps)")
    print(f"est peak sps ~ {est_peak_sps:.0f} (limit {args.max_sps:.0f})")
    print(f"update rate  = {fs:.1f} Hz (dt={dt*1000:.1f} ms)")
    print("Ctrl-C to stop.")

    try:
        while True:
            now = time.time()
            if t_end is not None and now >= t_end:
                break

            t = now - t0

            # Desired velocity v(t) = d/dt Σ Ai sin(2πfi t + phii)
            v_mm_s = 0.0
            for i in range(3):
                if A[i] == 0.0 or F[i] == 0.0:
                    continue
                w = 2.0 * math.pi * F[i]
                v_mm_s += A[i] * w * math.cos(w * t + PHI[i])

            # Convert to step rate (Hz)
            sps_cmd = abs(v_mm_s) * k
            if sps_cmd < args.deadband_sps:
                sps_cmd = 0.0
            sps_cmd = clamp(sps_cmd, 0.0, args.max_sps)

            # Slew limit to prevent abrupt frequency jumps
            dsps_max = max(0.0, args.slew_sps_per_s) * dt
            sps = clamp(sps_cmd, sps_prev - dsps_max, sps_prev + dsps_max)
            sps_prev = sps

            # Determine direction from velocity sign
            logical_dir = 1 if v_mm_s >= 0.0 else 0
            logical_dir ^= dir_invert

            # Reversal handling: force PWM off before changing DIR
            if last_dir is None:
                pi.write(args.dir, logical_dir)
                time.sleep(args.dir_setup_us / 1e6)
                last_dir = logical_dir
            elif logical_dir != last_dir:
                # ramp to zero (or just cut) for one tick, then flip
                pi.hardware_PWM(args.step, 0, 0)
                time.sleep(dt)
                pi.write(args.dir, logical_dir)
                time.sleep(args.dir_setup_us / 1e6)
                last_dir = logical_dir
                # after flip, keep sps as-is (slew limiting will bring it up smoothly)

            # Apply STEP PWM
            if sps <= 0.0:
                pi.hardware_PWM(args.step, 0, 0)
                steps_delivered = 0.0
            else:
                freq = int(round(sps))
                if freq < 1:
                    pi.hardware_PWM(args.step, 0, 0)
                    steps_delivered = 0.0
                else:
                    pi.hardware_PWM(args.step, freq, args.duty)
                    steps_delivered = float(freq) * dt  # approx

            # Update estimated position for safety bound
            # Convert "logical_dir" back to physical sign (+ means increasing x)
            phys_dir = (logical_dir ^ dir_invert)
            sign = 1.0 if phys_dir == 1 else -1.0
            s_est += sign * steps_delivered
            x_est = s_est / k
            if abs(x_est) > HALF_TRAVEL_MM + 1.0:
                raise RuntimeError(
                    f"Travel safety trip: estimated x={x_est:.2f} mm exceeds ±{HALF_TRAVEL_MM:.2f} mm."
                )

            # Pace the loop
            # (Use sleep to target dt; pigpio PWM runs in hardware regardless.)
            loop_end = time.time()
            remaining = dt - (loop_end - now)
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass
    finally:
        pi.hardware_PWM(args.step, 0, 0)
        pi.write(args.dir, 0)
        pi.stop()
        print("stopped")


if __name__ == "__main__":
    main()

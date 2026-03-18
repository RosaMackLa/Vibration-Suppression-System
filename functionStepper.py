#!/usr/bin/env python3
import argparse
import math
import time
import threading
import pigpio

# Mechanics defaults
DEFAULT_PULLEY_D_MM = 11.84
DEFAULT_PULSES_PER_REV = 3200

TRAVEL_TOTAL_MM = 150.0
HALF_TRAVEL_MM = TRAVEL_TOTAL_MM / 2.0

DEFAULT_STEP_BCM = 13   # physical pin 33 (PWM1) — avoids snd_bcm2835 conflict on PWM0/GPIO18
DEFAULT_DIR_BCM = 16

def steps_per_mm(pulley_d_mm: float, pulses_per_rev: float) -> float:
    return pulses_per_rev / (math.pi * pulley_d_mm)

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def parse_args():
    p = argparse.ArgumentParser("VSS function stepper (hardware PWM STEP)")
    p.add_argument("--step", type=int, default=DEFAULT_STEP_BCM, help="BCM STEP pin (13 or 19 for PWM1; avoids audio driver conflict)")
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

    p.add_argument("--rate", type=float, default=1000.0, help="Update rate (Hz) for position integrator and velocity computation")
    p.add_argument("--dur", type=float, default=0.0, help="Duration seconds, 0=run forever")

    p.add_argument("--max_sps", type=float, default=8000.0, help="Max steps/sec clamp")
    p.add_argument("--deadband_sps", type=float, default=0.0, help="Below this step rate, set PWM off (prevents chatter)")

    # FIX: PWM frequency change threshold — only issue a hardware_PWM call when the
    # commanded frequency changes by more than this many steps/sec. At 1000Hz update
    # rate, hammering pigpiod with ~1000 socket calls/sec causes it to queue and
    # process commands in bursts, producing brief motor stalls. A 50 sps threshold
    # is imperceptible to motion quality (0.6 mm/s velocity resolution at k=86)
    # but reduces pigpiod call rate by 80%+ during slow parts of the sine wave.
    p.add_argument("--pwm_freq_threshold", type=int, default=50,
                   help="Min frequency change (sps) to trigger a hardware_PWM call. "
                        "Reduces pigpiod socket load. 0=update every tick.")

    p.add_argument("--slew_sps_per_s", type=float, default=500000.0, help="Max step-rate change per second")

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

    if not (0 <= args.duty <= 1_000_000):
        raise SystemExit("ERROR: --duty must be 0..1_000_000")

    k = steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)

    A = [args.a1, args.a2, args.a3]
    F = [args.f1, args.f2, args.f3]
    PHI = [math.radians(args.phi1), math.radians(args.phi2), math.radians(args.phi3)]

    const_mode = args.const_sps > 0.0
    const_freq = int(round(clamp(args.const_sps, 0.0, args.max_sps)))
    const_dir = 1 if args.const_dir else 0
    if args.dir_invert:
        const_dir ^= 1

    # Conservative peak velocity bound
    vmax_mm_s = 0.0
    for i in range(3):
        if A[i] != 0.0 and F[i] != 0.0:
            vmax_mm_s += abs(A[i]) * (2.0 * math.pi * abs(F[i]))
    est_peak_sps = vmax_mm_s * k
    if not const_mode and est_peak_sps > args.max_sps + 1e-9:
        raise SystemExit(
            f"ERROR: estimated peak step rate ~{est_peak_sps:.0f} sps exceeds "
            f"max_sps={args.max_sps:.0f}.\n"
            f"Lower amplitudes/frequencies or raise max_sps only after testing."
        )

    # Warn if slew rate is too low
    if not const_mode:
        required_slew = 0.0
        for i in range(3):
            if A[i] != 0.0 and F[i] != 0.0:
                required_slew += abs(A[i]) * (2.0 * math.pi * abs(F[i]))**2 * k
        if required_slew > args.slew_sps_per_s + 1e-9:
            print(
                f"WARNING: --slew_sps_per_s={args.slew_sps_per_s:.0f} is too low "
                f"(need >= {required_slew:.0f} sps/s).\n"
                f"  Suggested: --slew_sps_per_s {int(required_slew * 1.25)}"
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
    pi.hardware_PWM(args.step, 0, 0)

    dir_invert = 1 if args.dir_invert else 0
    last_dir = None

    s_est = 0.0
    sps_prev = 0.0

    # PWM state — last frequency actually SENT to pigpiod (not just computed)
    last_sent_freq = 0
    last_sent_duty = 0
    pwm_is_on = False

    # Diagnostic: count pigpiod calls per status period
    pwm_call_count = 0

    t0 = time.monotonic()
    deadline = t0
    t_end = (t0 + args.dur) if args.dur > 0 else None
    last_now = None

    # Shared state for background thread
    status_lock = threading.Lock()
    status_state = {
        "t": 0.0, "freq": 0, "v_mm_s": 0.0, "x_est": 0.0,
        "last_dir": 0, "pwm_is_on": False, "last_sent_freq": 0,
        "actual_dt_ms": 0.0, "pwm_calls": 0, "running": True,
    }

    def background_thread():
        """Handles status printing and pigpiod keepalive off the motor loop."""
        status_period = (1.0 / args.status_hz) if args.status_hz and args.status_hz > 0 else None
        next_status = time.monotonic() if status_period else None
        next_keepalive = time.monotonic() + 2.0

        while True:
            now = time.monotonic()

            # Keepalive: prevent pigpiod ~5s inactivity timeout
            if now >= next_keepalive:
                try:
                    pi.get_current_tick()
                except Exception:
                    pass
                next_keepalive = now + 2.0

            # Status print
            if next_status is not None and now >= next_status:
                with status_lock:
                    s = dict(status_state)
                if const_mode:
                    print(
                        f"t={s['t']:7.3f}s  sps={s['freq']:6d}  "
                        f"dir={s['last_dir']}  pwm_on={int(s['pwm_is_on'])}  "
                        f"sent_freq={s['last_sent_freq']}  "
                        f"actual_dt={s['actual_dt_ms']:.2f}ms  "
                        f"pigpio_calls={s['pwm_calls']}"
                    )
                else:
                    print(
                        f"t={s['t']:7.3f}s  sps={s['freq']:6d}  "
                        f"v={s['v_mm_s']:+8.2f} mm/s  x_est={s['x_est']:+7.2f} mm  "
                        f"dir={s['last_dir']}  pwm_on={int(s['pwm_is_on'])}  "
                        f"sent_freq={s['last_sent_freq']}  "
                        f"actual_dt={s['actual_dt_ms']:.2f}ms  "
                        f"pigpio_calls={s['pwm_calls']}"
                    )
                next_status += status_period

            with status_lock:
                still_running = status_state["running"]
            if not still_running:
                break

            time.sleep(0.005)

    bg = threading.Thread(target=background_thread, daemon=True)
    bg.start()

    print(f"steps_per_mm = {k:.4f}")
    print(f"half_travel = {HALF_TRAVEL_MM:.1f} mm (~{HALF_TRAVEL_MM*k:.0f} steps)")
    if not const_mode:
        print(f"est peak sps ~ {est_peak_sps:.0f} (limit {args.max_sps:.0f})")
    print(f"update rate = {fs:.1f} Hz (dt={dt*1000:.2f} ms)")
    print(f"pwm_freq_threshold = {args.pwm_freq_threshold} sps")
    print(f"Tip: run with 'sudo chrt -f 50 python3 ...' for lower scheduling jitter")
    print("Ctrl-C to stop.")

    def pwm_off():
        nonlocal last_sent_freq, last_sent_duty, pwm_is_on, pwm_call_count
        if pwm_is_on:
            try:
                pi.hardware_PWM(args.step, 0, 0)
                pwm_call_count += 1
            except Exception:
                pass
            pwm_is_on = False
            last_sent_freq = 0
            last_sent_duty = 0

    def pwm_set(freq_hz: int, duty: int):
        """Only call hardware_PWM if frequency changed by more than pwm_freq_threshold.
        This dramatically reduces pigpiod socket load without affecting motion quality."""
        nonlocal last_sent_freq, last_sent_duty, pwm_is_on, pwm_call_count
        if freq_hz <= 0:
            pwm_off()
            return
        freq_changed = abs(freq_hz - last_sent_freq) > args.pwm_freq_threshold
        duty_changed = (duty != last_sent_duty)
        just_turned_on = not pwm_is_on
        if freq_changed or duty_changed or just_turned_on:
            try:
                pi.hardware_PWM(args.step, freq_hz, duty)
                pwm_is_on = True
                last_sent_freq = freq_hz
                last_sent_duty = duty
                pwm_call_count += 1
            except Exception:
                pass

    try:
        while True:
            now = time.monotonic()
            if t_end is not None and now >= t_end:
                break

            actual_dt = (now - last_now) if last_now is not None else dt
            actual_dt = min(actual_dt, dt * 5.0)
            last_now = now

            t = now - t0

            if const_mode:
                v_mm_s = 0.0
                sps_cmd = float(const_freq)
                logical_dir = const_dir
            else:
                v_mm_s = 0.0
                for i in range(3):
                    if A[i] == 0.0 or F[i] == 0.0:
                        continue
                    w = 2.0 * math.pi * F[i]
                    v_mm_s += A[i] * w * math.cos(w * t + PHI[i])

                logical_dir = 1 if v_mm_s >= 0.0 else 0
                logical_dir ^= dir_invert

                sps_cmd = abs(v_mm_s) * k
                if sps_cmd < args.deadband_sps:
                    sps_cmd = 0.0
                sps_cmd = clamp(sps_cmd, 0.0, args.max_sps)

            # Slew limit
            dsps_max = max(0.0, args.slew_sps_per_s) * dt
            sps = clamp(sps_cmd, sps_prev - dsps_max, sps_prev + dsps_max)
            sps_prev = sps

            # Handle DIR flips
            if last_dir is None:
                try:
                    pi.write(args.dir, logical_dir)
                except Exception:
                    pass
                time.sleep(args.dir_setup_us / 1e6)
                last_dir = logical_dir
            elif logical_dir != last_dir:
                pwm_off()
                if args.dir_blanking_us > 0:
                    time.sleep(args.dir_blanking_us / 1e6)
                try:
                    pi.write(args.dir, logical_dir)
                except Exception:
                    pass
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
                    steps_delivered = float(freq) * actual_dt

            # Update estimated position for safety bound
            phys_dir = (logical_dir ^ dir_invert)
            sign = 1.0 if phys_dir == 1 else -1.0
            s_est += sign * steps_delivered
            x_est = s_est / k
            if (not args.no_travel_safety) and (not const_mode):
                if abs(x_est) > HALF_TRAVEL_MM + 1.0:
                    raise RuntimeError(
                        f"Travel safety trip: estimated x={x_est:.2f} mm "
                        f"exceeds ±{HALF_TRAVEL_MM:.2f} mm."
                    )

            # Push state to background thread (non-blocking)
            if status_lock.acquire(blocking=False):
                try:
                    status_state["t"] = t
                    status_state["freq"] = freq
                    status_state["v_mm_s"] = v_mm_s
                    status_state["x_est"] = x_est
                    status_state["last_dir"] = last_dir
                    status_state["pwm_is_on"] = pwm_is_on
                    status_state["last_sent_freq"] = last_sent_freq
                    status_state["actual_dt_ms"] = actual_dt * 1000.0
                    status_state["pwm_calls"] = pwm_call_count
                    pwm_call_count = 0  # reset counter each status snapshot
                finally:
                    status_lock.release()

            # Deadline-based pacing
            deadline += dt
            sleep_s = deadline - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                deadline = time.monotonic()

    except KeyboardInterrupt:
        pass
    finally:
        with status_lock:
            status_state["running"] = False
        bg.join(timeout=1.0)
        try:
            pi.hardware_PWM(args.step, 0, 0)
            pi.write(args.dir, 0)
        except Exception:
            pass
        finally:
            pi.stop()
        print("stopped")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import argparse, math, time
import pigpio

# ---- Your hardware constants ----
PULLEY_D_MM = 10.9
PULSES_PER_REV = 3200
TRAVEL_TOTAL_MM = 150.0
HALF_TRAVEL_MM = TRAVEL_TOTAL_MM / 2.0

def steps_per_mm():
    return PULSES_PER_REV / (math.pi * PULLEY_D_MM)

def build_wave_from_step_sequence(step_pin, dir_pin, dir_invert, s_des_list, Ts_us, pulse_us, max_sps):
    """
    Convert desired step positions at fixed sample time Ts into pigpio pulses.
    Returns (pulses, s_final).
    """
    pulses = []
    s_curr = s_des_list[0]  # we'll start from the first desired position
    dir_state = None

    # max steps allowed in one sample slot (safety)
    Ts = Ts_us * 1e-6
    max_steps_per_sample = max(1, int(math.floor(max_sps * Ts)))

    for s_des in s_des_list[1:]:
        ds = s_des - s_curr
        if ds > max_steps_per_sample: ds = max_steps_per_sample
        if ds < -max_steps_per_sample: ds = -max_steps_per_sample

        if ds == 0:
            pulses.append(pigpio.pulse(0, 0, Ts_us))
            continue

        direction = 1 if ds > 0 else 0
        direction ^= (1 if dir_invert else 0)

        # DIR edge + small setup time
        if dir_state is None or direction != dir_state:
            if direction:
                pulses.append(pigpio.pulse(1 << dir_pin, 0, 2))
            else:
                pulses.append(pigpio.pulse(0, 1 << dir_pin, 2))
            dir_state = direction

        steps = abs(ds)
        slot_us = Ts_us

        spacing_us = max(pulse_us + 2, slot_us // steps)
        used = 0
        for _ in range(steps):
            pulses.append(pigpio.pulse(1 << step_pin, 0, pulse_us))
            low = max(0, spacing_us - pulse_us)
            pulses.append(pigpio.pulse(0, 1 << step_pin, low))
            used += spacing_us

        rem = slot_us - used
        if rem > 0:
            pulses.append(pigpio.pulse(0, 0, rem))

        s_curr += steps if ds > 0 else -steps

    return pulses, s_curr

def recenter(pi, step_pin, dir_pin, dir_invert, s_curr, recenter_sps=1500, pulse_us=8):
    """
    Move back to s=0 at a slow, safe constant step rate.
    This prevents wall hits between frequency tests even if you had minor rounding error.
    """
    if s_curr == 0:
        return 0

    direction = 1 if s_curr < 0 else 0  # need to step opposite sign of s_curr
    direction ^= (1 if dir_invert else 0)
    pi.write(dir_pin, 1 if direction else 0)
    time.sleep(0.02)

    nsteps = abs(s_curr)
    period_us = int(1e6 / recenter_sps)
    high_us = pulse_us
    low_us = max(4, period_us - high_us)

    # Use pigpio wave repeat in chunks
    chunk_steps = min(nsteps, 5000)
    while nsteps > 0:
        take = min(nsteps, chunk_steps)
        pi.wave_clear()
        # Build a wave that produces 'take' steps
        pulses = []
        for _ in range(take):
            pulses.append(pigpio.pulse(1 << step_pin, 0, high_us))
            pulses.append(pigpio.pulse(0, 1 << step_pin, low_us))
        pi.wave_add_generic(pulses)
        wid = pi.wave_create()
        if wid < 0:
            # fallback: gpio_trigger if wave too big
            for _ in range(take):
                pi.gpio_trigger(step_pin, high_us, 1)
                time.sleep(period_us * 1e-6)
        else:
            pi.wave_send_once(wid)
            while pi.wave_tx_busy():
                time.sleep(0.001)
            pi.wave_delete(wid)
        nsteps -= take

    return 0

def main():
    ap = argparse.ArgumentParser("Sine frequency sweep (oscillatory, zero-mean) for VSS stepper")
    ap.add_argument("--step", type=int, default=6, help="BCM STEP pin")
    ap.add_argument("--dir",  type=int, default=16, help="BCM DIR pin")
    ap.add_argument("--dir_invert", action="store_true", help="Invert DIR polarity")

    ap.add_argument("--A", type=float, required=True, help="Amplitude (mm, peak)")
    ap.add_argument("--f_start", type=float, default=1.0)
    ap.add_argument("--f_stop",  type=float, default=80.0)
    ap.add_argument("--f_step",  type=float, default=2.0)

    ap.add_argument("--hold", type=float, default=2.0, help="Seconds per frequency")
    ap.add_argument("--rate", type=float, default=4000.0, help="Trajectory sample rate (Hz)")
    ap.add_argument("--max_sps", type=float, default=12000.0, help="Max steps/s safety clamp")
    ap.add_argument("--pulse_us", type=int, default=8, help="STEP high time (us)")
    ap.add_argument("--recenter_sps", type=float, default=1500.0, help="Recentering step rate (steps/s)")
    args = ap.parse_args()

    k = steps_per_mm()

    # Travel safety (worst-case bound for single tone is just |A| <= half travel)
    if abs(args.A) > HALF_TRAVEL_MM + 1e-9:
        raise SystemExit(f"ERROR: A={args.A} mm exceeds half travel {HALF_TRAVEL_MM} mm.")

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("ERROR: pigpio not connected (is pigpiod running?).")

    pi.set_mode(args.step, pigpio.OUTPUT)
    pi.set_mode(args.dir,  pigpio.OUTPUT)
    pi.write(args.step, 0)

    print(f"steps_per_mm = {k:.4f}")
    print(f"Amplitude A = {args.A} mm (peak)")
    print("Stop with Ctrl-C at first sign of missed steps / instability.")
    print("f (Hz) | est peak sps | notes")

    s_curr = 0  # tracked position in steps (software)

    try:
        f = args.f_start
        while f <= args.f_stop + 1e-12:
            # Conservative peak step-rate estimate for a sine:
            # sps_peak = k * 2*pi*f*A
            sps_peak = k * (2.0 * math.pi * f * abs(args.A))

            if sps_peak > args.max_sps:
                print(f"{f:6.2f} | {sps_peak:11.0f} | exceeds max_sps={args.max_sps:.0f} -> stopping sweep")
                break

            # Choose samples per cycle: high enough to represent sine, low enough to fit pigpio wave limits.
            # Limit samples so wave doesn't explode in size.
            Ns = int(max(60, min(600, round(args.rate / f))))  # 60..600 samples per cycle
            T = 1.0 / f
            Ts = T / Ns
            Ts_us = max(50, int(round(Ts * 1e6)))  # don't go below 50us tick

            # Build exactly one cycle starting at t=0 and ending at t=T (return to same phase)
            # Use Ns+1 points so final equals initial; we will feed all points.
            s_des = []
            for n in range(Ns + 1):
                t = n * (T / Ns)
                x = args.A * math.sin(2.0 * math.pi * f * t)  # mm
                s_des.append(int(round(k * x)))

            # Ensure we start each frequency test from center (0 steps) to prevent wall hits.
            s_curr = recenter(pi, args.step, args.dir, args.dir_invert, s_curr,
                              recenter_sps=args.recenter_sps, pulse_us=args.pulse_us)

            # Now run for hold seconds by repeating the one-cycle waveform
            pulses, s_end = build_wave_from_step_sequence(
                args.step, args.dir, args.dir_invert,
                s_des, Ts_us, args.pulse_us, args.max_sps
            )

            # Safety: if this cycle doesn't end where it started due to clamping/quantization, we will recenter after hold anyway.
            pi.wave_clear()
            pi.wave_add_generic(pulses)
            wid = pi.wave_create()
            if wid < 0:
                print(f"{f:6.2f} | {sps_peak:11.0f} | wave too big -> lower --rate or reduce A/f")
                break

            cycles = max(1, int(round(args.hold * f)))  # integer cycles
            print(f"{f:6.2f} | {sps_peak:11.0f} | Ns={Ns}, Ts_us={Ts_us}, cycles={cycles}")

            # Send repeats using wave_chain (repeat wid cycles times)
            # wave_chain format: [255, 0, wid, 255, 1, x, y] where repeats = x + 256*y
            x = cycles & 0xFF
            y = (cycles >> 8) & 0xFF
            pi.wave_chain([255, 0, wid, 255, 1, x, y])

            while pi.wave_tx_busy():
                time.sleep(0.01)

            pi.wave_delete(wid)

            # Update software position estimate (best guess)
            s_curr = s_end

            # Recenter between frequencies to guarantee no wall hits
            s_curr = recenter(pi, args.step, args.dir, args.dir_invert, s_curr,
                              recenter_sps=args.recenter_sps, pulse_us=args.pulse_us)

            f += args.f_step

    except KeyboardInterrupt:
        pass
    finally:
        pi.wave_tx_stop()
        pi.wave_clear()
        pi.write(args.step, 0)
        pi.stop()
        print("stopped")

if __name__ == "__main__":
    main()

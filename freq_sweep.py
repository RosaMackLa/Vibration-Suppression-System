#!/usr/bin/env python3
"""
freq_sweep.py
Sinusoidal position command with continuous frequency sweep (chirp)
STEP = GPIO18 (default)
DIR  = GPIO16
"""

import argparse
import math
import time
import pigpio


PULLEY_D_MM = 10.9
PULSES_PER_REV = 6400
TRAVEL_TOTAL_MM = 150.0
HALF_TRAVEL_MM = TRAVEL_TOTAL_MM / 2.0


def steps_per_mm():
    circumference = math.pi * PULLEY_D_MM
    return PULSES_PER_REV / circumference


def parse_args():
    p = argparse.ArgumentParser("VSS frequency sweep (chirp)")
    p.add_argument("--step", type=int, default=18)
    p.add_argument("--dir", type=int, default=16)
    p.add_argument("--dir_invert", action="store_true")

    p.add_argument("--amp_mm", type=float, default=2.0)
    p.add_argument("--f_start", type=float, default=1.0)
    p.add_argument("--f_end", type=float, default=80.0)
    p.add_argument("--sweep_time", type=float, default=20.0)

    p.add_argument("--rate", type=float, default=2000.0)
    p.add_argument("--max_sps", type=float, default=40000.0)
    p.add_argument("--pulse_us", type=int, default=8)

    p.add_argument("--repeat", type=int, default=0)
    p.add_argument("--gap", type=float, default=0.5)

    # NEW: print rate limiter
    p.add_argument("--print_rate", type=float, default=5.0,
                   help="Status prints per second")

    return p.parse_args()


def chirp_phase(t, f0, f1, T):
    k = (f1 - f0) / T
    return 2.0 * math.pi * (f0 * t + 0.5 * k * t * t)


def chirp_freq(t, f0, f1, T):
    return f0 + (f1 - f0) * t / T


def build_wave_for_steps(pi, step_pin, dir_pin, dir_invert,
                         s_curr, s_des_list, Ts, max_sps, pulse_us):

    pulses = []
    dir_state = None
    slot_us = int(Ts * 1e6)
    max_steps_per_sample = max(1, int(math.floor(max_sps * Ts)))

    for s_des in s_des_list:
        ds = s_des - s_curr

        if ds > max_steps_per_sample:
            ds = max_steps_per_sample
        elif ds < -max_steps_per_sample:
            ds = -max_steps_per_sample

        if ds == 0:
            pulses.append(pigpio.pulse(0, 0, slot_us))
            continue

        direction = 1 if ds > 0 else 0
        direction ^= (1 if dir_invert else 0)

        if dir_state is None or direction != dir_state:
            if direction == 1:
                pulses.append(pigpio.pulse(1 << dir_pin, 0, 2))
            else:
                pulses.append(pigpio.pulse(0, 1 << dir_pin, 2))
            dir_state = direction

        steps = abs(ds)
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

    pi.wave_clear()
    pi.wave_add_generic(pulses)
    wid = pi.wave_create()
    return wid, s_curr


def main():
    args = parse_args()
    k_mm = steps_per_mm()
    fs = args.rate
    Ts = 1.0 / fs

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio not connected")

    pi.set_mode(args.step, pigpio.OUTPUT)
    pi.set_mode(args.dir, pigpio.OUTPUT)
    pi.write(args.step, 0)

    chunk_s = 0.10
    N = int(round(chunk_s * fs))

    s_curr = 0
    sweep_count = 0

    last_print = 0
    print_interval = 1.0 / args.print_rate

    try:
        while True:
            sweep_count += 1
            if args.repeat > 0 and sweep_count > args.repeat:
                break

            t0 = time.time()
            t_end = t0 + args.sweep_time

            while True:
                now = time.time()
                if now >= t_end:
                    break

                t_into = now - t0
                s_des_list = []

                for n in range(N):
                    t = t_into + n * Ts
                    if t > args.sweep_time:
                        t = args.sweep_time

                    phi = chirp_phase(t, args.f_start, args.f_end, args.sweep_time)
                    x_mm = args.amp_mm * math.sin(phi)
                    s_des_list.append(int(round(k_mm * x_mm)))

                wid, s_curr = build_wave_for_steps(
                    pi, args.step, args.dir, args.dir_invert,
                    s_curr, s_des_list, Ts,
                    args.max_sps, args.pulse_us
                )

                pi.wave_send_once(wid)
                while pi.wave_tx_busy():
                    time.sleep(0.001)
                pi.wave_delete(wid)

                # ---- STATUS PRINTING ----
                if now - last_print > print_interval:
                    f_inst = chirp_freq(t_into, args.f_start,
                                        args.f_end, args.sweep_time)

                    v_inst = args.amp_mm * 2 * math.pi * f_inst
                    sps_inst = k_mm * v_inst

                    print(
                        f"Sweep {sweep_count} | "
                        f"t={t_into:6.2f}s | "
                        f"f={f_inst:7.2f} Hz | "
                        f"vel≈{v_inst:7.2f} mm/s | "
                        f"steps/s≈{sps_inst:8.0f}"
                    )

                    last_print = now

            print(f"Completed sweep {sweep_count}")

    except KeyboardInterrupt:
        pass
    finally:
        pi.write(args.step, 0)
        pi.wave_clear()
        pi.stop()
        print("stopped")


if __name__ == "__main__":
    main()

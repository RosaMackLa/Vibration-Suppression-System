#!/usr/bin/env python3
import argparse, math, time
import pigpio

# ---- Hardware constants (your setup) ----
PULLEY_D_MM = 10.9
PULSES_PER_REV = 3200
TRAVEL_TOTAL_MM = 150.0           # total safe travel
HALF_TRAVEL_MM = TRAVEL_TOTAL_MM/2

STEP_PIN_DEFAULT = 6              # BCM
DIR_PIN_DEFAULT  = 16             # BCM

# ---- Helpers ----
def steps_per_mm():
    C = math.pi * PULLEY_D_MM
    return PULSES_PER_REV / C     # steps/mm

def parse_args():
    p = argparse.ArgumentParser("VSS 3-tone sinusoidal position command (STEP/DIR via pigpio)")
    p.add_argument("--step", type=int, default=STEP_PIN_DEFAULT, help="BCM STEP pin")
    p.add_argument("--dir",  type=int, default=DIR_PIN_DEFAULT,  help="BCM DIR pin")
    p.add_argument("--dir_invert", action="store_true", help="Invert DIR polarity")

    p.add_argument("--a1", type=float, default=0.0, help="Amplitude 1 (mm, peak)")
    p.add_argument("--f1", type=float, default=0.0, help="Frequency 1 (Hz)")
    p.add_argument("--a2", type=float, default=0.0, help="Amplitude 2 (mm, peak)")
    p.add_argument("--f2", type=float, default=0.0, help="Frequency 2 (Hz)")
    p.add_argument("--a3", type=float, default=0.0, help="Amplitude 3 (mm, peak)")
    p.add_argument("--f3", type=float, default=0.0, help="Frequency 3 (Hz)")

    p.add_argument("--phi1", type=float, default=0.0, help="Phase 1 (deg)")
    p.add_argument("--phi2", type=float, default=0.0, help="Phase 2 (deg)")
    p.add_argument("--phi3", type=float, default=0.0, help="Phase 3 (deg)")

    p.add_argument("--rate", type=float, default=2000.0, help="Trajectory sample rate (Hz). 1000–4000 typical.")
    p.add_argument("--dur",  type=float, default=0.0, help="Duration seconds. 0 = run forever.")
    p.add_argument("--max_sps", type=float, default=8000.0, help="Max steps/s safety clamp.")
    p.add_argument("--pulse_us", type=int, default=8, help="STEP high pulse width (us).")
    return p.parse_args()

def main():
    args = parse_args()
    k = steps_per_mm()

    A = [args.a1, args.a2, args.a3]
    f = [args.f1, args.f2, args.f3]
    phi = [math.radians(args.phi1), math.radians(args.phi2), math.radians(args.phi3)]

    # ---- Safety: travel bound using worst-case sum of amplitudes ----
    if sum(abs(x) for x in A) > HALF_TRAVEL_MM + 1e-9:
        raise SystemExit(
            f"ERROR: sum(|Ai|)={sum(abs(x) for x in A):.3f} mm exceeds half-travel {HALF_TRAVEL_MM:.3f} mm.\n"
            f"Reduce amplitudes or re-center with homing."
        )

    # ---- Safety: step-rate bound (conservative upper bound) ----
    vmax_mm_s = sum(abs(A[i]) * (2.0 * math.pi * f[i]) for i in range(3))
    est_peak_sps = k * vmax_mm_s
    if est_peak_sps > args.max_sps:
        raise SystemExit(
            f"ERROR: estimated peak step rate ~{est_peak_sps:.0f} steps/s exceeds max_sps={args.max_sps:.0f}.\n"
            f"Reduce amplitudes/frequencies or raise max_sps only after testing."
        )

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("ERROR: pigpio not connected. Is pigpiod running? (systemctl status pigpiod)")

    pi.set_mode(args.step, pigpio.OUTPUT)
    pi.set_mode(args.dir,  pigpio.OUTPUT)
    pi.write(args.step, 0)

    dir_invert = 1 if args.dir_invert else 0

    fs = args.rate
    Ts = 1.0 / fs

    # We stream in chunks to keep wave sizes reasonable.
    chunk_s = 0.10  # 100 ms chunks
    N = max(10, int(round(chunk_s * fs)))

    # State: current commanded step position (integer)
    s_curr = 0
    t0 = time.time()
    t_end = t0 + args.dur if args.dur > 0 else None

    print(f"steps_per_mm = {k:.4f}")
    print(f"half_travel  = {HALF_TRAVEL_MM:.1f} mm (~{HALF_TRAVEL_MM*k:.0f} steps)")
    print(f"est peak sps ~ {est_peak_sps:.0f} (limit {args.max_sps:.0f})")
    print("Ctrl-C to stop.")

    try:
        while True:
            now = time.time()
            if t_end is not None and now >= t_end:
                break

            # Build desired step positions for this chunk on a uniform grid
            s_des_list = []
            base_t = now - t0
            for n in range(N):
                t = base_t + n * Ts
                x = (A[0]*math.sin(2*math.pi*f[0]*t + phi[0]) +
                     A[1]*math.sin(2*math.pi*f[1]*t + phi[1]) +
                     A[2]*math.sin(2*math.pi*f[2]*t + phi[2]))
                s_des_list.append(int(round(k * x)))

            # Convert step position sequence -> timed pulses
            pulses = []
            dir_state = None
            max_steps_per_sample = max(1, int(math.floor(args.max_sps * Ts)))

            for s_des in s_des_list:
                ds = s_des - s_curr

                # Clamp how many steps we allow in one sample (safety)
                if ds > max_steps_per_sample: ds = max_steps_per_sample
                if ds < -max_steps_per_sample: ds = -max_steps_per_sample

                if ds == 0:
                    pulses.append(pigpio.pulse(0, 0, int(Ts*1e6)))
                    continue

                direction = 1 if ds > 0 else 0
                direction ^= dir_invert

                if dir_state is None or direction != dir_state:
                    # Set DIR; give it a small setup time (2 us)
                    if direction == 1:
                        pulses.append(pigpio.pulse(1 << args.dir, 0, 2))
                    else:
                        pulses.append(pigpio.pulse(0, 1 << args.dir, 2))
                    dir_state = direction

                steps = abs(ds)
                slot_us = int(Ts * 1e6)

                # Spread multiple steps inside the sample slot
                spacing_us = max(args.pulse_us + 2, slot_us // steps)
                used = 0
                for _ in range(steps):
                    pulses.append(pigpio.pulse(1 << args.step, 0, args.pulse_us))
                    low = max(0, spacing_us - args.pulse_us)
                    pulses.append(pigpio.pulse(0, 1 << args.step, low))
                    used += spacing_us

                rem = slot_us - used
                if rem > 0:
                    pulses.append(pigpio.pulse(0, 0, rem))

                s_curr += steps if ds > 0 else -steps

            # Send as one wave
            pi.wave_clear()
            pi.wave_add_generic(pulses)
            wid = pi.wave_create()
            if wid < 0:
                raise RuntimeError("wave_create failed (too many pulses). Lower --rate or increase chunk length.")

            pi.wave_send_once(wid)
            while pi.wave_tx_busy():
                time.sleep(0.001)
            pi.wave_delete(wid)

    except KeyboardInterrupt:
        pass
    finally:
        pi.write(args.step, 0)
        pi.stop()
        print("stopped")

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import argparse
import subprocess
import sys
import time
import numpy as np

import freq_Isolator as fi


def parse_args():
    p = argparse.ArgumentParser("VSS Master: identify top 3 freqs, then drive 3-tone stepper command")

    # Identification
    p.add_argument("--id_duration", type=float, default=10.0,
                   help="Seconds of accel data to collect for identification")
    p.add_argument("--id_fs", type=float, default=800.0,
                   help="Target sampling rate during identification (Hz)")
    p.add_argument("--fmin", type=float, default=5.0,
                   help="Min frequency to consider (Hz)")
    p.add_argument("--fmax", type=float, default=400.0,
                   help="Max frequency to consider (Hz)")
    p.add_argument("--simulate", action="store_true",
                   help="Use simulated accel in freq_Isolator")

    # Mapping accel -> motion
    p.add_argument("--total_amp_mm", type=float, default=2.0,
                   help="Total displacement budget (mm peak) to distribute across 3 tones (before max_sps scaling)")
    p.add_argument("--amp_floor", type=float, default=0.01,
                   help="Floor on relative amplitudes before normalization (prevents near-zero tone from disappearing)")
    p.add_argument("--sps_margin", type=float, default=0.9,
                   help="Safety margin: scale so estimated peak_sps <= margin*max_sps")
    p.add_argument("--weight_mode", choices=["rel", "rel_over_f", "rel_over_f2"], default="rel_over_f",
                   help="How to distribute displacement across tones before scaling to max_sps")

    # Mechanics for step reate prediction
    p.add_argument("--steps_per_mm", type=float, default=None,
                   help="Direct steps/mm conversion for your axis (overrides motor/microsteps/mm_per_rev)")
    p.add_argument("--motor_steps", type=int, default=200,
                   help="Full steps per rev of motor (usually 200)")
    p.add_argument("--microsteps", type=int, default=16,
                   help="Microsteps setting on the driver (e.g., 16)")
    p.add_argument("--mm_per_rev", type=float, default=8.0,
                   help="Axis travel per motor revolution (lead screw lead, or pulley travel per rev)")

    # Stepper execution
    p.add_argument("--step_pin", type=int, default=6, help="BCM STEP pin")
    p.add_argument("--dir_pin", type=int, default=16, help="BCM DIR pin")
    p.add_argument("--dir_invert", action="store_true", help="Invert DIR polarity")
    p.add_argument("--traj_rate", type=float, default=2000.0,
                   help="Trajectory sample rate for wave generation (Hz)")
    p.add_argument("--run_dur", type=float, default=15.0,
                   help="How long to run the 3-tone command (s). 0=forever")
    p.add_argument("--max_sps", type=float, default=8000.0,
                   help="Max steps/s clamp (software safety)")
    p.add_argument("--pulse_us", type=int, default=8,
                   help="STEP high pulse width (us)")

    # Looping
    p.add_argument("--loop", action="store_true", help="Repeat identify->run forever")
    p.add_argument("--settle_s", type=float, default=1.0, help="Pause between cycles (s)")

    return p.parse_args()


def get_steps_per_mm(args) -> float:
    if args.steps_per_mm is not None:
        return float(args.steps_per_mm)
    steps_per_rev = args.motor_steps * args.microsteps
    return float(steps_per_rev / args.mm_per_rev)


def estimate_peak_sps(f_hz, a_mm, steps_per_mm) -> float:
    f = np.array(f_hz, dtype=float)
    A = np.array(a_mm, dtype=float)
    peak_mm_per_s = float(np.sum(2.0 * np.pi * f * A))
    return peak_mm_per_s * float(steps_per_mm)


def relamps_to_mm(rel_amps, f_hz, total_amp_mm, steps_per_mm, max_sps,
                  amp_floor=0.01, margin=0.9, weight_mode="rel_over_f"):
    """
    1) Allocate displacement across tones using rel_amps and frequency-aware weighting
    2) Predict conservative peak step rate bound
    3) Scale all amplitudes down uniformly to satisfy max_sps with a margin
    """
    rel = np.array(rel_amps, dtype=float)
    f = np.array(f_hz, dtype=float)

    rel = np.maximum(rel, amp_floor)
    f_safe = np.maximum(f, 1e-6)

    if weight_mode == "rel":
        w = rel
    elif weight_mode == "rel_over_f":
        w = rel / f_safe
    elif weight_mode == "rel_over_f2":
        w = rel / (f_safe ** 2)
    else:
        w = rel / f_safe

    w = w / np.sum(w)
    A = total_amp_mm * w

    est = estimate_peak_sps(f_hz, A.tolist(), steps_per_mm)

    scale = 1.0
    limit = margin * max_sps
    if est > limit and est > 0:
        scale = limit / est
        A = A * scale

    return A.tolist(), est, scale


def run_function_stepper(args, f_hz, a_mm):
    cmd = [
        sys.executable, "functionStepper.py",
        "--step", str(args.step_pin),
        "--dir", str(args.dir_pin),
        "--rate", str(args.traj_rate),
        "--dur", str(args.run_dur),
        "--max_sps", str(args.max_sps),
        "--pulse_us", str(args.pulse_us),
        "--a1", str(a_mm[0]), "--f1", str(f_hz[0]),
        "--a2", str(a_mm[1]), "--f2", str(f_hz[1]),
        "--a3", str(a_mm[2]), "--f3", str(f_hz[2]),
        "--phi1", "0.0", "--phi2", "0.0", "--phi3", "0.0",
    ]
    if args.dir_invert:
        cmd.append("--dir_invert")

    print("\nLaunching stepper command:")
    print("  freqs (Hz):", [round(x, 3) for x in f_hz])
    print("  amps  (mm):", [round(x, 4) for x in a_mm])
    print("  cmd:", " ".join(cmd), "\n")

    subprocess.run(cmd, check=False)


def identify_top3(args):
    x, fs_meas = fi.collect_time_series(args.id_duration, args.id_fs, args.simulate)
    freqs, amps = fi.single_sided_fft_amplitude(x, fs_meas, window="hann")
    peaks = fi.pick_top_peaks(
        freqs=freqs,
        amps=amps,
        k=3,
        min_freq_hz=args.fmin,
        max_freq_hz=args.fmax,
        guard_bins=2,
    )

    if len(peaks) < 1:
        raise RuntimeError("No peaks found in band. Increase duration, widen band, or check sensor signal.")

    while len(peaks) < 3:
        peaks.append(peaks[-1])

    f_hz = [p.freq_hz for p in peaks[:3]]
    rel = [p.rel_amp for p in peaks[:3]]
    return fs_meas, f_hz, rel


def main():
    args = parse_args()
    steps_per_mm = get_steps_per_mm(args)

    print(f"Mechanics: steps_per_mm={steps_per_mm:.3f} "
          f"(motor_steps={args.motor_steps}, microsteps={args.microsteps}, mm_per_rev={args.mm_per_rev})")
    print(f"Amplitude policy: weight_mode={args.weight_mode}, total_amp_mm={args.total_amp_mm}, "
          f"max_sps={args.max_sps}, margin={args.sps_margin}")

    while True:
        fs_meas, f_hz, rel = identify_top3(args)

        a_mm, est_peak_sps_pre_scale, scale = relamps_to_mm(
            rel_amps=rel,
            f_hz=f_hz,
            total_amp_mm=args.total_amp_mm,
            steps_per_mm=steps_per_mm,
            max_sps=args.max_sps,
            amp_floor=args.amp_floor,
            margin=args.sps_margin,
            weight_mode=args.weight_mode,
        )

        est_peak_sps_post = estimate_peak_sps(f_hz, a_mm, steps_per_mm)

        print(f"\nID results: fs_meas={fs_meas:.2f} Hz | band=[{args.fmin},{args.fmax}] Hz")
        for i in range(3):
            print(f"  {i+1}) f={f_hz[i]:8.3f} Hz | rel_amp={rel[i]:.4f} | cmd_amp={a_mm[i]:.4f} mm")
        print(f"Step-rate estimate: pre_scale≈{est_peak_sps_pre_scale:.1f} sps | "
              f"scale={scale:.3f} | post_scale≈{est_peak_sps_post:.1f} sps")

        run_function_stepper(args, f_hz, a_mm)

        if not args.loop:
            break
        time.sleep(max(0.0, args.settle_s))


if __name__ == "__main__":
    main()

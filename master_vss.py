#!/usr/bin/env python3
import argparse
import math
import subprocess
import sys
import time

import numpy as np

# Import your existing frequency ID code
import freq_Isolator as fi


def parse_args():
    p = argparse.ArgumentParser("VSS Master: identify top 3 freqs, then drive 3-tone stepper command")

    # Identification
    p.add_argument("--id_duration", type=float, default=10.0, help="Seconds of accel data to collect for identification")
    p.add_argument("--id_fs", type=float, default=800.0, help="Target sampling rate during identification (Hz)")
    p.add_argument("--fmin", type=float, default=5.0, help="Min frequency to consider (Hz)")
    p.add_argument("--fmax", type=float, default=400.0, help="Max frequency to consider (Hz)")
    p.add_argument("--simulate", action="store_true", help="Use simulated accel in freq_Isolator")

    # Mapping accel -> motion
    p.add_argument("--total_amp_mm", type=float, default=10.0,
                   help="Total displacement budget (mm peak) to distribute across 3 tones")
    p.add_argument("--amp_floor", type=float, default=0.05,
                   help="If a rel_amp is below this, clamp it up (prevents zeroing a tone)")

    # Stepper execution (passed through to functionStepper.py)
    p.add_argument("--step_pin", type=int, default=6, help="BCM STEP pin")
    p.add_argument("--dir_pin", type=int, default=16, help="BCM DIR pin")
    p.add_argument("--dir_invert", action="store_true", help="Invert DIR polarity")
    p.add_argument("--traj_rate", type=float, default=2000.0, help="Trajectory sample rate for wave generation (Hz)")
    p.add_argument("--run_dur", type=float, default=15.0, help="How long to run the 3-tone command (s). 0=forever")
    p.add_argument("--max_sps", type=float, default=8000.0, help="Max steps/s clamp")
    p.add_argument("--pulse_us", type=int, default=8, help="STEP high pulse width (us)")

    # Looping
    p.add_argument("--loop", action="store_true", help="Repeat identify->run forever")
    p.add_argument("--settle_s", type=float, default=1.0, help="Pause between cycles (s)")

    return p.parse_args()


def relamps_to_mm(rel_amps, total_amp_mm, amp_floor):
    w = np.array(rel_amps, dtype=float)
    w = np.maximum(w, amp_floor)
    w = w / np.sum(w)
    return (total_amp_mm * w).tolist()


def run_function_stepper(args, f_hz, a_mm):
    # For now: phases = 0. Later: estimate phase with least-squares and pass phi_k.
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
    print("  amps  (mm):", [round(x, 3) for x in a_mm])
    print("  cmd:", " ".join(cmd), "\n")

    # Run and stream output live
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

    # Ensure we always have 3 tones (pad with the strongest peak if fewer found)
    while len(peaks) < 3:
        peaks.append(peaks[-1])

    f_hz = [p.freq_hz for p in peaks[:3]]
    rel = [p.rel_amp for p in peaks[:3]]
    return fs_meas, f_hz, rel


def main():
    args = parse_args()

    while True:
        fs_meas, f_hz, rel = identify_top3(args)
        a_mm = relamps_to_mm(rel, args.total_amp_mm, args.amp_floor)

        print(f"\nID results: fs_meas={fs_meas:.2f} Hz | band=[{args.fmin},{args.fmax}] Hz")
        for i in range(3):
            print(f"  {i+1}) f={f_hz[i]:8.3f} Hz | rel_amp={rel[i]:.3f} | cmd_amp={a_mm[i]:.3f} mm")

        run_function_stepper(args, f_hz, a_mm)

        if not args.loop:
            break
        time.sleep(max(0.0, args.settle_s))


if __name__ == "__main__":
    main()

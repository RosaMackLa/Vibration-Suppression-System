#!/usr/bin/env python3
"""
functionStepper_wave.py — VSS sinusoidal stepper, waveform (DMA) architecture.

WHAT CHANGED vs functionStepper.py:
  - Replaced hardware_PWM-per-tick with pre-computed DMA waveforms.
  - Each ~50 ms chunk of exact step timings is handed to pigpiod once;
    the DMA engine plays it back with nanosecond precision, fully independent
    of the CPU scheduler. No per-step IPC during playback.
  - DIR changes encoded directly in the waveform pulse sequence, so there
    is no blanking sleep in the main loop — the driver sees the correct timing
    without the Python process needing to be scheduled at that moment.
  - Chunk handoff rate ~20/sec vs ~400-1000/sec, eliminating the socket
    backlog that caused brief stalls and hard-stop reversals.

REMOVED args (no longer needed):
  --pwm_freq_threshold  — was a workaround for IPC overload; not needed here
  --rate / --slew_sps_per_s — no tick loop; timing computed analytically
  --duty                — DMA waveform uses fixed pulse width, not duty cycle

ADDED args:
  --chunk_ms  — waveform chunk duration (default 50 ms)

USAGE (identical to before):
  sudo chrt -f 50 python3 functionStepper_wave.py --a1 10 --f1 5
  sudo chrt -f 50 python3 functionStepper_wave.py --a1 8 --f1 3 --a2 4 --f2 7
"""

import argparse
import math
import time
import threading
import pigpio

# ── Mechanics defaults ─────────────────────────────────────────────────────────
DEFAULT_PULLEY_D_MM   = 11.84
DEFAULT_PULSES_PER_REV = 3200
TRAVEL_TOTAL_MM       = 150.0
HALF_TRAVEL_MM        = TRAVEL_TOTAL_MM / 2.0

DEFAULT_STEP_BCM = 13   # PWM1 / GPIO13 — avoids snd_bcm2835 conflict on PWM0/GPIO18
DEFAULT_DIR_BCM  = 16

# STEP pulse high time (µs). DM556 minimum is 2.5 µs; 5 µs gives comfortable margin.
PULSE_HIGH_US = 5


# ── Helpers ────────────────────────────────────────────────────────────────────
def steps_per_mm(pulley_d_mm: float, pulses_per_rev: float) -> float:
    return pulses_per_rev / (math.pi * pulley_d_mm)


# ── Argument parsing ───────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser("VSS function stepper (waveform DMA)")
    p.add_argument("--step",       type=int,   default=DEFAULT_STEP_BCM,
                   help="BCM STEP pin (13 or 19 for PWM1)")
    p.add_argument("--dir",        type=int,   default=DEFAULT_DIR_BCM,
                   help="BCM DIR pin")
    p.add_argument("--dir_invert", action="store_true", help="Invert DIR polarity")

    p.add_argument("--pulley_d_mm",    type=float, default=DEFAULT_PULLEY_D_MM)
    p.add_argument("--pulses_per_rev", type=float, default=DEFAULT_PULSES_PER_REV)

    # Up to 3 tones: x(t) = Σ Aᵢ·sin(2πfᵢt + φᵢ)
    for n in "123":
        p.add_argument(f"--a{n}",   type=float, default=0.0, help=f"Amplitude {n} (mm peak)")
        p.add_argument(f"--f{n}",   type=float, default=0.0, help=f"Frequency {n} (Hz)")
        p.add_argument(f"--phi{n}", type=float, default=0.0, help=f"Phase {n} (deg)")

    p.add_argument("--dur",     type=float, default=0.0,    help="Duration (s), 0=forever")
    p.add_argument("--max_sps", type=float, default=8000.0, help="Max steps/sec clamp")
    p.add_argument("--deadband_sps", type=float, default=20.0,
                   help="Below this sps, pause stepping. No-op delay pulses keep timing correct.")

    p.add_argument("--chunk_ms", type=float, default=50.0,
                   help="Waveform chunk duration (ms). "
                        "Longer = less IPC overhead. "
                        "Also the maximum latency from Ctrl-C to motor stop.")

    p.add_argument("--dir_setup_us",    type=int, default=20,
                   help="DIR-to-STEP setup time (µs)")
    p.add_argument("--dir_blanking_us", type=int, default=200,
                   help="STEP-off blanking before DIR flip (µs)")

    p.add_argument("--status_hz",        type=float, default=2.0,
                   help="Status print rate (Hz), 0=off")
    p.add_argument("--no_travel_safety", action="store_true",
                   help="Disable travel safety bound")
    return p.parse_args()


# ── Core: waveform chunk computation ──────────────────────────────────────────
def compute_chunk(
    t_start: float,
    chunk_s: float,
    A, F, PHI,
    k: float,
    max_sps: float,
    deadband_sps: float,
    step_pin: int,
    dir_pin: int,
    last_dir: int,
    dir_invert: bool,
    dir_blanking_us: int,
    dir_setup_us: int,
):
    """
    Compute a waveform chunk covering [t_start, t_start + chunk_s).

    Every step's inter-pulse gap encodes the instantaneous velocity.  DIR
    changes include blanking + setup delays directly in the pulse sequence —
    the DMA engine handles their timing with no Python involvement.

    Near zero-crossings (sps < deadband_sps), no steps are emitted but
    no-op delay pulses preserve the wall-clock timing so the next chunk
    starts in phase.

    Returns
    -------
    pulses    : list[pigpio.pulse]  ready for wave_add_generic()
    end_dir   : int                 DIR pin state at chunk boundary
    t_end     : float               = t_start + chunk_s  (phase clock)
    net_steps : int                 signed step count (+1 per phys-positive step)
    """
    STEP_BIT     = 1 << step_pin
    DIR_BIT      = 1 << dir_pin
    MIN_PERIOD_US = PULSE_HIGH_US + 1   # absolute floor on step period

    pulses      = []
    t           = t_start
    t_end       = t_start + chunk_s
    current_dir = last_dir
    net_steps   = 0

    while t < t_end:
        # ── Instantaneous velocity (mm/s) ─────────────────────────────────
        v = 0.0
        for i in range(3):
            if A[i] != 0.0 and F[i] != 0.0:
                w = 2.0 * math.pi * F[i]
                v += A[i] * w * math.cos(w * t + PHI[i])

        sps = min(abs(v) * k, max_sps)

        # ── Near zero-crossing: no step, preserve time ────────────────────
        if sps < deadband_sps:
            # Clamp to remaining chunk time; minimum 1 µs
            wait_us = min(1000, max(1, int((t_end - t) * 1e6)))
            pulses.append(pigpio.pulse(0, 0, wait_us))
            t += wait_us / 1e6
            continue

        # ── Direction ─────────────────────────────────────────────────────
        logical_dir = (1 if v >= 0.0 else 0) ^ (1 if dir_invert else 0)

        if logical_dir != current_dir:
            # Blank STEP, flip DIR, wait setup — all in the pulse stream
            if dir_blanking_us > 0:
                pulses.append(pigpio.pulse(0, 0, dir_blanking_us))
            if logical_dir == 1:
                pulses.append(pigpio.pulse(DIR_BIT, 0, dir_setup_us))
            else:
                pulses.append(pigpio.pulse(0, DIR_BIT, dir_setup_us))
            current_dir = logical_dir

        # ── Step pulse ────────────────────────────────────────────────────
        step_period_us = max(MIN_PERIOD_US, int(round(1e6 / sps)))
        pulses.append(pigpio.pulse(STEP_BIT, 0, PULSE_HIGH_US))
        pulses.append(pigpio.pulse(0, STEP_BIT, step_period_us - PULSE_HIGH_US))

        # Physical direction (undo dir_invert to get physical carriage sign)
        phys_dir  = current_dir ^ (1 if dir_invert else 0)
        net_steps += 1 if phys_dir == 1 else -1
        t         += step_period_us / 1e6

    return pulses, current_dir, t_end, net_steps


def wave_duration_s(pulses) -> float:
    """Exact wall-clock duration of a pulse list (µs → seconds)."""
    return sum(p.delay for p in pulses) / 1_000_000.0


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    args      = parse_args()
    k         = steps_per_mm(args.pulley_d_mm, args.pulses_per_rev)
    dir_invert = args.dir_invert
    chunk_s   = args.chunk_ms / 1000.0

    A   = [args.a1, args.a2, args.a3]
    F   = [args.f1, args.f2, args.f3]
    PHI = [math.radians(args.phi1), math.radians(args.phi2), math.radians(args.phi3)]

    # ── Pre-flight peak velocity check ────────────────────────────────────────
    est_peak_sps = sum(
        abs(A[i]) * 2.0 * math.pi * abs(F[i])
        for i in range(3) if A[i] and F[i]
    ) * k
    if est_peak_sps > args.max_sps + 1e-9:
        raise SystemExit(
            f"ERROR: estimated peak ~{est_peak_sps:.0f} sps exceeds "
            f"--max_sps={args.max_sps:.0f}.  Lower amplitudes/frequencies or raise --max_sps."
        )

    # ── pigpio init ───────────────────────────────────────────────────────────
    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("ERROR: pigpiod not running. Start with: sudo systemctl start pigpiod")

    pi.set_mode(args.dir,  pigpio.OUTPUT)
    pi.set_mode(args.step, pigpio.OUTPUT)
    pi.set_mode(25, pigpio.OUTPUT)
    pi.write(25, 1)   # enable hardware backstop interlock
    pi.wave_clear()

    # Set DIR pin to match initial velocity direction before first pulse
    v0 = sum(
        A[i] * 2.0 * math.pi * F[i] * math.cos(PHI[i])
        for i in range(3) if A[i] and F[i]
    )
    init_dir = (1 if v0 >= 0.0 else 0) ^ (1 if dir_invert else 0)
    pi.write(args.dir, init_dir)
    time.sleep(args.dir_setup_us / 1e6)

    # ── Background thread: status + pigpiod keepalive ─────────────────────────
    status_lock  = threading.Lock()
    status_state = {
        "t": 0.0, "x_est": 0.0,
        "steps_last_chunk": 0, "actual_chunk_ms": 0.0,
        "running": True,
    }

    def bg_thread():
        period   = (1.0 / args.status_hz) if args.status_hz > 0 else None
        next_print = time.monotonic() if period else None
        next_ka    = time.monotonic() + 2.0

        while True:
            now = time.monotonic()

            if now >= next_ka:
                try:
                    pi.get_current_tick()
                except Exception:
                    pass
                next_ka = now + 2.0

            if period and now >= next_print:
                with status_lock:
                    s = dict(status_state)
                if not s["running"]:
                    break
                print(
                    f"t={s['t']:7.3f}s  "
                    f"x_est={s['x_est']:+7.2f} mm  "
                    f"steps/chunk={s['steps_last_chunk']:+5d}  "
                    f"chunk={s['actual_chunk_ms']:.1f} ms"
                )
                next_print += period

            with status_lock:
                if not status_state["running"]:
                    break

            time.sleep(0.005)

    bg = threading.Thread(target=bg_thread, daemon=True)
    bg.start()

    print(f"steps_per_mm   = {k:.4f}")
    print(f"half_travel    = {HALF_TRAVEL_MM:.1f} mm  (~{HALF_TRAVEL_MM*k:.0f} steps)")
    print(f"est peak sps   ~ {est_peak_sps:.0f}  (limit {args.max_sps:.0f})")
    print(f"chunk duration = {chunk_s*1000:.0f} ms")
    print("Tip: run with 'sudo chrt -f 50 python3 ...' for lower scheduling jitter")
    print("Ctrl-C to stop.")

    t_phase      = 0.0
    s_est        = 0.0
    last_dir     = init_dir
    t_wall_start = time.monotonic()
    t_wall_end   = (t_wall_start + args.dur) if args.dur > 0 else None

    # ── Compute and launch first chunk ────────────────────────────────────────
    pulses_curr, last_dir, t_phase, steps_curr = compute_chunk(
        t_phase, chunk_s, A, F, PHI, k,
        args.max_sps, args.deadband_sps,
        args.step, args.dir, last_dir, dir_invert,
        args.dir_blanking_us, args.dir_setup_us,
    )
    dur_curr = wave_duration_s(pulses_curr)
    pi.wave_add_generic(pulses_curr)
    wave_id_curr   = pi.wave_create()
    pi.wave_send_once(wave_id_curr)
    chunk_wall_start = time.monotonic()

    # ── Double-buffer main loop ───────────────────────────────────────────────
    #
    #   While chunk N plays via DMA, Python computes chunk N+1.
    #   When N finishes, N+1 starts immediately (< 100 µs gap from poll loop).
    #   N is then deleted. Repeat.
    #
    #   Position is updated with exact step counts once each chunk finishes,
    #   so s_est never drifts from quantisation or slew-lag the way the old
    #   dt-integration did.
    #
    try:
        while True:
            if t_wall_end and time.monotonic() >= t_wall_end:
                break

            # Compute next chunk while current one plays on DMA hardware
            pulses_next, dir_next, t_next, steps_next = compute_chunk(
                t_phase, chunk_s, A, F, PHI, k,
                args.max_sps, args.deadband_sps,
                args.step, args.dir, last_dir, dir_invert,
                args.dir_blanking_us, args.dir_setup_us,
            )
            dur_next = wave_duration_s(pulses_next)

            # Sleep until ~10 ms before current chunk ends, then poll
            sleep_s = dur_curr - (time.monotonic() - chunk_wall_start) - 0.010
            if sleep_s > 0:
                time.sleep(sleep_s)
            while pi.wave_tx_busy():
                time.sleep(0.0001)

            # ── Current chunk just finished ───────────────────────────────
            # Update position with exact steps from the completed chunk
            s_est += steps_curr
            x_est  = s_est / k

            if (not args.no_travel_safety) and abs(x_est) > HALF_TRAVEL_MM + 1.0:
                raise RuntimeError(
                    f"Travel safety trip: estimated x={x_est:.2f} mm "
                    f"exceeds ±{HALF_TRAVEL_MM:.2f} mm."
                )

            # ── Start next chunk immediately ──────────────────────────────
            pi.wave_add_generic(pulses_next)
            wave_id_next     = pi.wave_create()
            pi.wave_send_once(wave_id_next)
            chunk_wall_start = time.monotonic()

            # Free the waveform that just finished
            pi.wave_delete(wave_id_curr)

            # Push status (non-blocking: skip if bg thread holds lock)
            if status_lock.acquire(blocking=False):
                try:
                    status_state["t"]                = time.monotonic() - t_wall_start
                    status_state["x_est"]            = x_est
                    status_state["steps_last_chunk"] = steps_curr
                    status_state["actual_chunk_ms"]  = dur_next * 1000.0
                finally:
                    status_lock.release()

            # Rotate state for next iteration
            wave_id_curr = wave_id_next
            dur_curr     = dur_next
            steps_curr   = steps_next
            last_dir     = dir_next
            t_phase      = t_next

    except KeyboardInterrupt:
        pass

    finally:
        with status_lock:
            status_state["running"] = False
        bg.join(timeout=1.0)
        try:
            pi.wave_tx_stop()
        except Exception:
            pass
        try:
            pi.wave_clear()
            pi.write(args.step, 0)
            pi.write(args.dir,  0)
            pi.write(25, 0)
        except Exception:
            pass
        pi.stop()
        print("stopped.")


if __name__ == "__main__":
    main()

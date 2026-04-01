#!/usr/bin/env python3
"""
vss_lms_sim.py  —  Standalone LMS convergence simulator
════════════════════════════════════════════════════════
Tests the EXACT LMS update law from vss_controller.py without pigpio/hardware.
Run on any machine: python3 vss_lms_sim.py

PURPOSE
───────
Verify that the LMS update equations converge in simulation before committing
to a hardware run.  If it diverges here, there is a code bug.  If it converges
here but not on hardware, the failure is plant-model mismatch or hardware issue
— a completely different diagnostic.

PLANT MODEL
───────────
Unlike the built-in --simulate mode in vss_controller.py (which has no plant
phase and uses absolute-time phase instead of the accumulator), this sim uses:

  a_plant(t) = G * [C·cos(φ + φ_P) + S·sin(φ + φ_P)]

where φ is the phase accumulator (same as LMS thread) and φ_P is the plant
phase from plant_id.py.  This correctly tests phase-driven stability.

Equivalent to rotating the (C, S) phasor by φ_P before applying gain G:
  C_rot = C·cos(φ_P) − S·sin(φ_P)
  S_rot = C·sin(φ_P) + S·cos(φ_P)
  a_plant = G · (C_rot·cos(φ) + S_rot·sin(φ))

WHAT TO LOOK FOR
────────────────
  CONVERGING  : e_rms falls monotonically, A grows then plateaus, C/S settle
  DIVERGING   : A hits clamp instantly, C/S rotate continuously → µ too large
  PLATEAU     : e_rms stops falling before zero → wrong frequency or plant phase
                mismatch too large (|φ_P| > 90°)
  OSCILLATING : µ_omega too large, ω overshoots disturbance frequency
"""

import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ═══════════════════════════════════════════════════════════════════════════════
# PARAMETERS — edit these to match your intended hardware run
# ═══════════════════════════════════════════════════════════════════════════════

FS          = 800.0          # Control loop sample rate [Hz] — matches hardware
DT          = 1.0 / FS
SIM_DUR_S   = 120.0          # Simulation duration [s]

# LMS parameters — use exact values from your next hardware run flags
MU          = 1e-5           # Amplitude learning rate   --mu 1e-5
MU_OMEGA    = 3e-7           # Frequency learning rate   --mu_omega 3e-7
                             # Set to 0.0 to disable frequency tracking

# Disturbance (the DC motor on the shelf)
F_DIST      = 9.0            # True disturbance frequency [Hz] — center of safe window
A_DIST      = 3.0            # Disturbance amplitude [m/s²] — measured ~3 m/s² in prior runs

# Plant model — from plant_id.py results at ~10 Hz
PLANT_GAIN  = 5.2            # [m/s² per mm of counterweight displacement]
PLANT_PHASE_DEG = -25.0      # [degrees] — plant ID measured -20° to -35° in 8-11 Hz band
PLANT_PHASE = math.radians(PLANT_PHASE_DEG)

# Initial LMS state
C0, S0      = 0.0, 0.0       # Start from zero (matches hardware init_amp_gain=0)
F0          = F_DIST         # Initial frequency estimate [Hz]

# Safety clamp (matches hardware args)
MAX_AMP_PER_TONE = 20.0      # mm
MIN_FREQ         = 1.0       # Hz
MAX_FREQ         = 10.0      # Hz

# ═══════════════════════════════════════════════════════════════════════════════
# SIMULATION LOOP
# Exact copy of lms_thread_fn update equations — do not "fix" these
# ═══════════════════════════════════════════════════════════════════════════════

n_samples = int(SIM_DUR_S * FS)

# Storage arrays
t_arr   = np.zeros(n_samples)
C_arr   = np.zeros(n_samples)
S_arr   = np.zeros(n_samples)
A_arr   = np.zeros(n_samples)
phi_arr = np.zeros(n_samples)   # LMS phase accumulator value [rad]
e_arr   = np.zeros(n_samples)
f_arr   = np.zeros(n_samples)

# Initial state
C     = C0
S     = S0
omega = 2.0 * math.pi * F0
phi   = 0.0     # phase accumulator

print(f"VSS LMS Simulation")
print(f"  f_dist={F_DIST}Hz  A_dist={A_DIST}m/s²")
print(f"  Plant: G={PLANT_GAIN}  φ_P={PLANT_PHASE_DEG:.0f}°")
print(f"  µ={MU}  µ_omega={MU_OMEGA}  fs={FS}Hz  dur={SIM_DUR_S}s")
print(f"  Running {n_samples} samples...")

for i in range(n_samples):
    t = i * DT

    # ── Advance phase accumulator ─────────────────────────────────────────────
    # EXACT COPY from lms_thread_fn — do not change
    phi += omega * DT
    if phi > math.pi:
        phi -= 2.0 * math.pi

    # ── Disturbance signal ────────────────────────────────────────────────────
    a_dist = A_DIST * math.sin(2.0 * math.pi * F_DIST * t)

    # ── Plant response to actuator output ─────────────────────────────────────
    # Actuator: x(t) = C·cos(φ) + S·sin(φ)  [mm]
    # Plant applies gain G and phase shift φ_P to the actuator signal.
    # Implemented by rotating the (C,S) phasor by φ_P before evaluating:
    #   a_plant = G · (C_rot·cos(φ) + S_rot·sin(φ))
    C_rot   = C * math.cos(PLANT_PHASE) - S * math.sin(PLANT_PHASE)
    S_rot   = C * math.sin(PLANT_PHASE) + S * math.cos(PLANT_PHASE)
    a_plant = PLANT_GAIN * (C_rot * math.cos(phi) + S_rot * math.sin(phi))

    # ── Error = what the IMU would measure ───────────────────────────────────
    e = a_dist + a_plant

    # ── LMS gradient steps ────────────────────────────────────────────────────
    # EXACT COPY from lms_thread_fn — do not change
    cos_ref = math.cos(phi)
    sin_ref = math.sin(phi)

    C -= MU * e * cos_ref
    S -= MU * e * sin_ref

    # Frequency update
    if MU_OMEGA > 0:
        quad    = S * cos_ref - C * sin_ref   # quadrature signal
        omega  -= MU_OMEGA * e * quad
        f_hz    = omega / (2.0 * math.pi)
        f_hz    = max(MIN_FREQ, min(MAX_FREQ, f_hz))
        omega   = 2.0 * math.pi * f_hz

    # ── Amplitude clamp ───────────────────────────────────────────────────────
    # EXACT COPY from clamp_coeffs (single-tone case)
    A = math.sqrt(C**2 + S**2)
    if A > MAX_AMP_PER_TONE:
        r  = MAX_AMP_PER_TONE / A
        C *= r
        S *= r
        A  = MAX_AMP_PER_TONE

    # ── Log ───────────────────────────────────────────────────────────────────
    t_arr[i]   = t
    C_arr[i]   = C
    S_arr[i]   = S
    A_arr[i]   = A
    phi_arr[i] = phi
    e_arr[i]   = e
    f_arr[i]   = omega / (2.0 * math.pi)

print("  Done.\n")

# ── Quick convergence diagnostics ────────────────────────────────────────────
window_s  = 2.0
window    = int(window_s * FS)
e_rms     = np.array([
    np.sqrt(np.mean(e_arr[max(0, i - window):i + 1] ** 2))
    for i in range(n_samples)
])

e_rms_init  = e_rms[window]                   # after first 2s (transient excluded)
e_rms_final = e_rms[-1]
A_final     = A_arr[-1]
f_final     = f_arr[-1]
tau_idx     = next((i for i in range(window, n_samples)
                    if e_rms[i] < e_rms_init * 0.37), None)
tau_s       = tau_idx * DT if tau_idx else None

print(f"Diagnostics:")
print(f"  e_rms initial (t=2s) : {e_rms_init:.5f} m/s²")
print(f"  e_rms final          : {e_rms_final:.5f} m/s²")
print(f"  Reduction            : {20*np.log10(e_rms_final/e_rms_init + 1e-12):.1f} dB")
print(f"  Time constant τ      : {f'{tau_s:.1f}s' if tau_s else 'not reached (diverging?)'}")
print(f"  Final amplitude      : {A_final:.4f} mm")
print(f"  Final frequency      : {f_final:.4f} Hz  (true: {F_DIST} Hz)")
print(f"  Amplitude clamped?   : {'YES — check µ' if A_arr[window] >= MAX_AMP_PER_TONE * 0.99 else 'no'}")

# ── Plots ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
fig.suptitle(
    f'VSS LMS Convergence Sim\n'
    f'f={F_DIST}Hz  A_dist={A_DIST}m/s²  '
    f'Plant: G={PLANT_GAIN} φ_P={PLANT_PHASE_DEG:.0f}°  '
    f'µ={MU}  µω={MU_OMEGA}',
    fontsize=10
)

# Panel 1: Residual error
ax = axes[0]
ax.plot(t_arr, e_arr, alpha=0.2, color='gray', linewidth=0.4, label='e(t)')
ax.plot(t_arr, e_rms, color='crimson', linewidth=1.5, label=f'e_rms ({window_s:.0f}s window)')
ax.axhline(0, color='black', linewidth=0.5)
ax.set_ylabel('Error [m/s²]')
ax.set_title('Residual Error  (should fall monotonically if converging)')
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)

# Panel 2: Actuator amplitude
ax = axes[1]
ax.plot(t_arr, A_arr, color='steelblue', linewidth=1.5)
ax.axhline(MAX_AMP_PER_TONE, color='red', linestyle='--', linewidth=0.8,
           label=f'clamp = {MAX_AMP_PER_TONE}mm')
ax.set_ylabel('Amplitude [mm]')
ax.set_title('Actuator Amplitude |C + jS|  (should grow then plateau)')
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)

# Panel 3: C and S coefficients
ax = axes[2]
ax.plot(t_arr, C_arr, linewidth=1.0, label='C  (cos coeff)', color='royalblue')
ax.plot(t_arr, S_arr, linewidth=1.0, label='S  (sin coeff)', color='darkorange')
ax.axhline(0, color='black', linewidth=0.5)
ax.set_ylabel('[mm]')
ax.set_title('LMS Coefficients  (should settle; continuous rotation = diverging)')
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)

# Panel 4: Tracked frequency
ax = axes[3]
ax.plot(t_arr, f_arr, color='seagreen', linewidth=1.5)
ax.axhline(F_DIST, color='black', linestyle='--', linewidth=0.8,
           label=f'True f = {F_DIST} Hz')
ax.set_ylabel('Frequency [Hz]')
ax.set_xlabel('Time [s]')
ax.set_title('Tracked Frequency  (should converge to true freq if µ_omega > 0)')
ax.legend(loc='upper right', fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = 'lms_sim_convergence.png'
plt.savefig(out, dpi=150)
print(f'\nPlot saved: {out}')

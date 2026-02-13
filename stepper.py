from gpiozero import OutputDevice
import time

# ----------------------
# Pin definitions (BCM)
# ----------------------
STEP_PIN = 6
DIR_PIN = 16

step_pin = OutputDevice(STEP_PIN)
dir_pin = OutputDevice(DIR_PIN)

# ----------------------
# Motor settings
# ----------------------
FULL_STEPS_PER_REV = 200
MICROSTEP = 16
STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEP
STEPS_PER_DEGREE = STEPS_PER_REV / 360.0

current_steps = 0

# ----------------------
# Pulse settings
# ----------------------
STEP_DELAY = 0.0005   # controls speed (lower = faster)

def move_to(target_steps):
    global current_steps

    delta = target_steps - current_steps
    direction = 1 if delta > 0 else 0
    dir_pin.value = direction

    steps = abs(delta)

    for _ in range(steps):
        step_pin.on()
        time.sleep(0.00001)  # pulse width
        step_pin.off()
        time.sleep(STEP_DELAY)

    current_steps = target_steps


print("Enter angle (0–360 degrees):")

try:
    while True:
        target_angle = float(input("> "))

        if 0 <= target_angle <= 360:
            target_steps = int(target_angle * STEPS_PER_DEGREE)
            print(f"Moving to {target_angle}° ({target_steps} steps)")
            move_to(target_steps)
        else:
            print("Invalid angle. Enter 0–360 only.")

except KeyboardInterrupt:
    pass

finally:
    step_pin.off()
    dir_pin.off()
    step_pin.close()
    dir_pin.close()
    print("GPIO released.")


from gpiozero import OutputDevice
import time

def test_level_shifter(gpio_pin=6, interval=1.0):
    pin = OutputDevice(gpio_pin)

    print("Toggling GPIO. Measure HV side with multimeter.")
    print("CTRL+C to stop.")

    try:
        while True:
            pin.on()
            print("HIGH")
            time.sleep(interval)

            pin.off()
            print("LOW")
            time.sleep(interval)

    except KeyboardInterrupt:
        pin.off()
        print("Test stopped.")

    finally:
        pin.off()
        pin.close()
        print("GPIO released.")

# ---- Run test ----
test_level_shifter(gpio_pin=6, interval=3.0)

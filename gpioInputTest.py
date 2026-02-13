from gpiozero import OutputDevice, Button
import time

# GPIO pins (BCM numbering)
BUTTON_PIN = 17

# Set up pins
button = Button(BUTTON_PIN, pull_up=False)

# Drive constant HIGH
print("Press the button. CTRL+C to exit.")

try:
    while True:
        if button.is_pressed:
            print("BUTTON PRESSED")
        else:
            print("button idle")
        time.sleep(0.3)

except KeyboardInterrupt:
    print("Test ended.")

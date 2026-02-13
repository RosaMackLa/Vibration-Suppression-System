import time
import board
import busio
from adafruit_lsm6ds.lsm6dso32 import LSM6DSO32

i2c = busio.I2C(board.SCL, board.SDA)
imu = LSM6DSO32(i2c)

while True:
    ax, ay, az = imu.acceleration
    gx, gy, gz = imu.gyro

    print(
        f"Accel (m/s^2): {ax:7.3f}, {ay:7.3f}, {az:7.3f} | "
        f"Gyro (rad/s): {gx:7.3f}, {gy:7.3f}, {gz:7.3f}"
    )

    time.sleep(0.05)

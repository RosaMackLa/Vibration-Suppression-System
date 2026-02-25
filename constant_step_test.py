#!/usr/bin/env python3
import time
import pigpio

STEP = 6
DIR  = 16

SPS = 2000   # steps per second
RUN_S = 10

pi = pigpio.pi()
assert pi.connected

pi.set_mode(STEP, pigpio.OUTPUT)
pi.set_mode(DIR,  pigpio.OUTPUT)

pi.write(DIR, 1)

# 50% duty square wave on STEP
half_us = int(1_000_000 / (2*SPS))
pi.wave_clear()
pi.wave_add_generic([
    pigpio.pulse(1<<STEP, 0, half_us),
    pigpio.pulse(0, 1<<STEP, half_us),
])
wid = pi.wave_create()
pi.wave_send_repeat(wid)

time.sleep(RUN_S)

pi.wave_tx_stop()
pi.wave_delete(wid)
pi.stop()
print("done")

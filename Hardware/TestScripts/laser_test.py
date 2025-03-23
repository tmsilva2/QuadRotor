import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685
import time

SENSOR_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)
testSec = 20
delaySec = 0.0001

try:
    counter = 0
    t = 0
    reset = 1
    while t < testSec:
        blocked = GPIO.input(SENSOR_PIN)==GPIO.HIGH
        if blocked and reset:
            print(f"Laser blocked! Count: {counter}")
            counter += 1
            reset = 0 # only count this block once
        elif not blocked:
            print(f"Laser unblocked!")
            reset = 1
            
        t += delaySec
        time.sleep(delaySec)
except KeyboardInterrupt:
    print("\nExiting...")
    GPIO.cleanup()





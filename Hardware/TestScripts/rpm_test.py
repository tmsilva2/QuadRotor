import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685
import time




# Init I2Cand PCA9685
i2c = busio.I2C(board.SCL,board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50 #ESC frequency = 50 Hz
min_value = 1000
max_value = 1500

SENSOR_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN,GPIO.IN,pull_up_down=GPIO.PUD_UP)


def set_esc_pulsewidth(pulse,channel):
    if pulse > max_value:
        pulse = max_value
    elif pulse < min_value:
        pulse = min_value
        
    pwm_val = int((pulse/20000)*65535) # Assuming 50 Hz and 16-bit driver resolution
    pca.channels[channel].duty_cycle = pwm_val

# Test PWM Output on Channel 0
print("Sending test PWM signal to PCA685 Channel 15...")


escChannel = 15 #pca9685 channels for ESCs
pulse = 1100

# First, arm the ESC with min pulsewidth 1000us
set_esc_pulsewidth(min_value,escChannel)
time.sleep(3)

set_esc_pulsewidth(pulse,escChannel)
time.sleep(1)
delaySec = 0.0005
testSec = 5

try:
    counter = 0
    t = 0
    reset = 1
    while t < testSec:
        blocked = GPIO.input(SENSOR_PIN)==GPIO.HIGH
        if blocked and reset:
            #print(f"Laser blocked! Count: {counter}")
            counter += 1
            reset = 0 # only count this block once
        elif not blocked:
            #print(f"Laser unblocked!")
            reset = 1
            
        t += delaySec
        time.sleep(delaySec)
except KeyboardInterrupt:
    print("\nExiting...")
    GPIO.cleanup()

# Calc rpm (assume 2 prop blades)
rpm = (counter/testSec/2)*60
print(f"Calculated RPM: {rpm}")

# Shutoff the PWM
set_esc_pulsewidth(0,escChannel)



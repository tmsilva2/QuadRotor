import pigpio
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
counter = 0

SENSOR_PIN = 17
pi = pigpio.pi()
pi.set_mode(SENSOR_PIN,pigpio.INPUT)
pi.set_pull_up_down(SENSOR_PIN,pigpio.PUD_UP)

def sensor_callback(gpio,level,tick):
    global counter
    if level == 1:
        counter += 1

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
pulse = 1800

# First, arm the ESC with min pulsewidth 1000us
set_esc_pulsewidth(min_value,escChannel)
time.sleep(3)

rpms = []
step = 50
testSec = 5

for pulse in range(min_value,max_value,step):
    counter = 0
    set_esc_pulsewidth(pulse,escChannel)
    time.sleep(1)
    
    cb = pi.callback(SENSOR_PIN,pigpio.RISING_EDGE,sensor_callback)
    print(f"Measuring RPM for pulse {pulse} for {testSec} seconds...")
    
    # Calc rpm (assume 2 prop blades)
    rpm = (counter/testSec/2)*60
    rpms.append(rpm)
    print(f"Calculated RPM: {rpm}")
    
    time.sleep(testSec)



# Shutoff the PWM and cleanup
set_esc_pulsewidth(0,escChannel)
cb.cancel()
pi.stop()



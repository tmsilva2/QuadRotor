import board
import busio
import adafruit_pca9685
import time




# Init I2Cand PCA9685
i2c = busio.I2C(board.SCL,board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50 #ESC frequency = 50 Hz
min_value = 1000
max_value = 2000

def set_esc_pulsewidth(pulse,channel):
    if pulse > max_value:
        pulse = max_value
    elif pulse < min_value:
        pulse = min_value
        
    pwm_val = int((pulse/20000)*65535) # Assuming 50 Hz and 16-bit driver resolution
    pca.channels[channel].duty_cycle = pwm_val

# Test PWM Output on Channel 0
print("Sending test PWM signal to PCA685 Channel 0...")


escChannels = [14,15] #pca9685 channels for ESCs


# First, arm the ESC with min pulsewidth 1000us
for channel in escChannels:
    set_esc_pulsewidth(min_value,channel)
time.sleep(3)

# Second, set the desired pulsewidth (1000us - 2000us)




for pulse in range(min_value,max_value,10):
    for channel in escChannels:
        set_esc_pulsewidth(pulse,channel)
    print(f"Throttle: {pulse}")
    time.sleep(0.25)

# Shutoff the PWM
for channel in escChannels:
    set_esc_pulsewidth(0,channel)




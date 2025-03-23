
import time
import board
import busio
import adafruit_pca9685

ESC = 12 # Connect the ESC to this GPIO pin

# Init I2Cand PCA9685
i2c = busio.I2C(board.SCL,board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)
pca.frequency = 50 #ESC frequency = 50 Hz
escChannels = [12,13,14,15] #pca9685 channels for ESCs
max_value = 2000
min_value = 1000

def set_esc_pulsewidth(pulse,channel):
    if pulse > max_value:
        pulse = max_value
    elif pulse < min_value:
        pulse = min_value
        
    pwm_val = int((pulse/20000)*65535) # Assuming 50 Hz and 16-bit driver resolution
    pca.channels[channel].duty_cycle = pwm_val

def calibrate():
    for esc in escChannels:
        set_esc_pulsewidth(0,esc)
    print("Disconnect the ESC power and wait")
    time.sleep(10)
    
    for esc in escChannels:
        set_esc_pulsewidth(max_value,esc)
    print("Connect the ESC power. You should hear beeps?")
    time.sleep(10)
    
    for esc in escChannels:
        set_esc_pulsewidth(min_value,esc)
    print("Setting min throttle...")
    time.sleep(10)
    
    for esc in escChannels:
        set_esc_pulsewidth(0,esc)
    print("Shutting off PWM...")
    time.sleep(10)
    
    for esc in escChannels:
        set_esc_pulsewidth(min_value,esc)
    print("Arming ESC...")
    time.sleep(10)
    
    for pulse in range(min_value,max_value,100):
        for esc in escChannels:
            set_esc_pulsewidth(pulse,esc)
        print(f"Setting ESCs to {pulse}...")
        time.sleep(5)
    
    
    for esc in escChannels:
        set_esc_pulsewidth(0,esc)
    print("Shutting off PWM...")

    
calibrate()
    
    
    

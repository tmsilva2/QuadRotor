
import time
import pigpio

ESC = 12 # Connect the ESC to this GPIO pin

pi = pigpio.pi()
pi.set_servo_pulsewidth(ESC,0)

max_value = 2000
min_value = 1000

def calibrate():
    pi.set_servo_pulsewidth(ESC,0)
    print("Disconnect the ESC power and wait")
    time.sleep(5)
    
    pi.set_servo_pulsewidth(ESC,max_value)
    print("Connect the ESC power. You should hear beeps?")
    time.sleep(15)
    
    pi.set_servo_pulsewidth(ESC,min_value)
    print("Setting min throttle...")
    time.sleep(15)
    
    pi.set_servo_pulsewidth(ESC,0)
    print("Shutting off PWM...")
    time.sleep(15)
    
    pi.set_servo_pulsewidth(ESC,min_value)
    print("Arming ESC...")
    time.sleep(15)
    
    print("Calibration Complete! Setting mid-range throttle...")
    pi.set_servo_pulsewidth(ESC,min_value)
    time.sleep(15)
    
calibrate()
    
    
    
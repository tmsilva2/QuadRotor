import pigpio
import time

pi = pigpio.pi()

ESC_GPIO = 12
max_value = 2000
min_value = 1000

pi.set_servo_pulsewidth(ESC_GPIO,min_value)
time.sleep(2)

print("Increasing throttle...")

for pulse in range(min_value,max_value,10):
	pi.set_servo_pulsewidth(ESC_GPIO,pulse)
	print(f"Throttle: {pulse}")
	time.sleep(0.25)

print("Stopping motor...")
pi.set_servo_pulsewidth(ESC_GPIO,1000)
time.sleep(2)

pi.stop()
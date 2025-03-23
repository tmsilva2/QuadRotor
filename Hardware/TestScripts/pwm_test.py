import RPi.GPIO as GPIO
import time

PWM_PIN = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN,GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN,50)

pwm.start(5.0)
#pwm.ChangeDutyCycle(100)

time.sleep(10)

pwm.stop()

GPIO.cleanup()

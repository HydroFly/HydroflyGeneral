import RPi.GPIO as GPIO
import time
from pin import setup

b=23
setup(b)
print("LED on")
GPIO.output(b,GPIO.HIGH)
time.sleep(5)
print("LED off")
GPIO.output(b,GPIO.LOW)

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
Pinno = 13
GPIO.setup(Pinno, GPIO.OUT, initial=GPIO.LOW)

state = True

while True:
    GPIO.output(Pinno,True)
    print("Hell Ya")
    time.sleep(0.5)
    GPIO.output(Pinno,False)
    print("Hell Na")
    time.sleep(0.5)
    

import RPi.GPIO as GPIO

def setup(a):
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(a,GPIO.OUT)

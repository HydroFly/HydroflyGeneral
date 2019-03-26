import subfile as sf
import time

import RPi.GPIO as GPIO

pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
def interrupt_handler(channel):
    print("Interrupt exception")
    global IsArmed
    IsArmed = 1

GPIO.add_event_detect(pin, GPIO.RISING, callback=interrupt_handler, bouncetime=200)

print("Entered main file")

sf.functioner()

Arm = 0
IsArmed = 0
TheVehicle = sf.Vehicle()

while(True):
    Arm = TheVehicle.ArmCheck(IsArmed)
    print("Looping: ", Arm)
    time.sleep(.5)

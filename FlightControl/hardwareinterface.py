# File to define every hardware interfaces
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM) #BCM naming convention instead of GPIO.board
#variable names based on BOARD convention. pin number uses BCM
gpio16 = 23
GPIO.setup(gpio16, GPIO.IN, pull_up_down = GPIO.PUD_UP)

gpio40 = 21
GPIO.setup(gpio40, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def interrupt_handler(channel):
    if (channel == gpio16):
        print("Interrupt exception: ", channel)
        global running
        running = False
    elif (channel == gpio40):
        print("Interrupt exception: ", channel)
        global SwitchArmed
        if(SwitchArmed ==1):
            interrupt_handler(gpio16)
        else:
            SwitchArmed = 1

#load GPIO event detections
def loadeventdetection():
    GPIO.add_event_detect(gpio16, GPIO.RISING, callback=interrupt_handler, bouncetime=200)
    GPIO.add_event_detect(gpio40, GPIO.RISING, callback=interrupt_handler, bouncetime=200)
    print("loaded")



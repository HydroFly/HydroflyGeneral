#Hardware setup: Button between pin 23 and ground. 
#only some pins work. Hardware limitation.
import RPi.GPIO as GPIO
import time

state = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def interrupt_handler(channel):
    global state

    print("interrupt handler")

    if state ==1:
        state = 0
        print("State reset by event on pin 23")
    else:
        state = 1
        print("State set to 1 by pin 23")
GPIO.add_event_detect(23, GPIO.RISING, callback=interrupt_handler, bouncetime=200)


while (True):
    time.sleep(0)

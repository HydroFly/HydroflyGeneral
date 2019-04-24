import time
from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

pin1 = 13
pin2 = 11
GPIO.setup(pin1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def interrupt_handler(channel):
    time.sleep(0.05)
    if GPIO.input(channel) == GPIO.LOW:
        return

    if (channel== pin1):
        if(GPIO.input(pin1) == False): # if switch closes 
            print("Unlocked 1!")
            global unlocked1
            unlocked1 = True
        else:
            print("Locked 1!")
            global unlocked1
            unlocked1 = False
 
    if (channel== pin2):  
        if(GPIO.input(pin2) == False): # if switch closes
            print("Unlocked 2!")
            global unlocked2
            unlocked2 = True
        else:
            print("Locked 2!")
            global unlocked2
            unlocked2 = False

GPIO.add_event_detect(pin1, GPIO.BOTH, callback=interrupt_handler, bouncetime=20)
GPIO.add_event_detect(pin2, GPIO.BOTH, callback=interrupt_handler, bouncetime=20)

running = False
unlocked1 = False
unlocked2 = False

print("on standby. Awaiting unlocking procedure to be complete.")
while(True):
    if(unlocked1 == True and unlocked2 == True): 
        #print("Fully Unlocked ;-)") 
        print("Unlocked1: ", unlocked1, "Unlocked2: ", unlocked2)
    elif(unlocked1 == False or unlocked2 == False):
        #print("Still locked, bish")
        print("Unlocked1: ", unlocked1, "Unlocked2: ", unlocked2)
    sleep(0.01)
    #print("monitoring")
    pass

print("Fully unlocked")

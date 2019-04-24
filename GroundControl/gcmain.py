import RPi.GPIO as GPIO
import time
from numpy import *

GPIO.setmode(GPIO.BOARD)

lockpins = [13, 11] #operator 1 and 2
lockledpins = [12, 16] #relay 4, relay 3
lockstatus = [0,0]
GPIO.setup(lockpins, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(lockledpins, GPIO.OUT, initial=GPIO.LOW)

mswitch_pins = [29, 31] #vehicle power, vehicle launch
mswitch_led_pins = [22,18] #relay 1, relay 2
mswitch_status = [0,0]
GPIO.setup(mswitch_pins, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(mswitch_led_pins, GPIO.OUT, initial=GPIO.LOW)

countdown_led_pins = [32, 36, 38] #relay 5, 6, 7
GPIO.setup(countdown_led_pins, GPIO.OUT, initial=GPIO.LOW)

abort_led_pin = 40 #relay 8
GPIO.setup(abort_led_pin, GPIO.OUT, initial=GPIO.LOW)

relaypin = 0#whatever we decide

def interrupt_handler(channel):
    time.sleep(0.05)
    #if GPIO.input(channel) == GPIO.LOW:
    #    print("DEBUG: SKIPPED!")
    #    return #not break, i think
    print("DEBUG: DETECTED! at", channel)

    if (channel == lockpins[0]):
        print("DEBUG, ",GPIO.input(lockpins[0]))
        if(GPIO.input(lockpins[0]) == True): # if switch is closed  #setting opposite condition
            print("Unlocked 1!")
            #global lockstatus
            lockstatus[0] = True
        else:
            print("Locked 1!")
            #global lockstatus
            lockstatus[0] = False
 
    if (channel == lockpins[1]):  
        if(GPIO.input(lockpins[1]) == True): # if switch closes
            print("Unlocked 2!")
            #global lockstatus
            lockstatus[1] = True
        else:
            print("Locked 2!")
            #global lockstatus
            lockstatus[1] = False

    #if (channel == mswitch_pins[0] ):
    if (channel == mswitch_pins[0] and (prod(lockstatus) == 1)):
        print("DEBUGGG MSWITCH")
        print(GPIO.input(mswitch_pins[0]))
        if(GPIO.input(mswitch_pins[0]) == True): 
            print("Vehicle PowerUP Command Sent")
            #global mswitch_status
            mswitch_status[0] = True
        else:
            print("Vehicle PowerDOWN Command Sent")
            #global mswitch_status
            mswitch_status[0] = False
    
    if (channel == mswitch_pins[1]):  
        if(GPIO.input(mswitch_pins[1]) == True and (prod(lockstatus)==1)):
            for i in range(0, len(countdown_led_pins)):
                GPIO.output(countdown_led_pins[i], True)
                time.sleep(0.5)
            launchtoggle()
            print("Vehicle Launch Command Sent")
            #global mswitch_status
            mswitch_status[1] = True
        else:
            print("Vehicle ABORT Command Sent")
            launchtoggle()
            #global mswitch_status
            mswitch_status[1] = False
            GPIO.output(abort_led_pin, False)
    print(lockstatus[0])
    for i in range(0, len(lockpins)):
        print("Changing status")
        GPIO.output(lockledpins[i], lockstatus[i])


    for i in range(0, len(mswitch_led_pins)):
        GPIO.output(mswitch_led_pins[i], mswitch_status[i])
        
def launchtoggle(relaypin):
    GPIO.output(relaypin, True)
    time.sleep(0.2)
    GPIO.output(relaypin, False)

GPIO.add_event_detect(lockpins[0], GPIO.BOTH, callback=interrupt_handler, bouncetime=20)
GPIO.add_event_detect(lockpins[1], GPIO.BOTH, callback=interrupt_handler, bouncetime=20)

GPIO.add_event_detect(mswitch_pins[0], GPIO.BOTH, callback=interrupt_handler, bouncetime=20)
GPIO.add_event_detect(mswitch_pins[1], GPIO.BOTH, callback=interrupt_handler, bouncetime=20)

while(prod(lockstatus) != 1):
    print("Lock 1 Satisfication: ", lockstatus[0], "Lock 2 Satisfaction: ", lockstatus[1])
    time.sleep(0.1)

print("Welcome to Hydrofly Ground Control!")

while(prod(mswitch_status) != 1):
    print("MSwitch 0", mswitch_status[0], "MSwitch 1", mswitch_status[1])
    time.sleep(0.1)

print("Setting up Flight Monitoring... or not")


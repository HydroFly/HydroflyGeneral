import RPi.GPIO as GPIO
import time

pinset = [12, 16, 18, 22]
PinStatus= [0, 0, 0, 0]
GPIO.setmode(GPIO.BOARD)

GPIO.setup(pinset, GPIO.OUT, initial=GPIO.LOW)

value = 1

print("     Testing Modes")
print("         1_ Manual Control")
print("         2_ Individual LED Speed Test")
print("         3_ All LED Flashing")
mode = int(input("  Input Testing Mode: "))

print(PinStatus)
while True:
    if mode == 1:
        value = input("Enter pin to turn on or off (1-4): ")

    elif mode == 2: 
        value = value + 1
        if value > 4:
            value = 1
    elif mode == 3:
        value = 0
    else:
        exit()
    

    pin = pinset[int(value)-1]
    if GPIO.input(pin) == 0:
        if mode == 3:
            for i in range(0, len(pinset)):
                GPIO.output(pinset[i], True)
        else:
            GPIO.output(pin, True)
        print("Pin ", pin, " activated")
    else:
        if mode == 3:
            for i in range(0,len(pinset)):
                GPIO.output(pinset[i], False)
        else:
            GPIO.output(pin, False)
        print("Pin ", pin, " deactivated")

    for i in range(0, len(pinset)):
        PinStatus[(i)] = GPIO.input(pinset[i])
        #print('looped')
    print(PinStatus)
    time.sleep(0.025)
#for improvement when i'm not on a time crunch:
#for all modes, have them go through an array of changes. (0001) (0100) or (1111) (0000), etc.
#mode 3 basically changes more than 1 at once

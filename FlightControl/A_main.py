#!/usr/bin/python3
import time
import csv
from time import sleep
import maxSonarTTY
import Adafruit_ADS1x15 
import RPi.GPIO as GPIO
import datetime
import os
import copy
import utilities as utils
import FlightController as FC
import numpy as np
import threading 



#import hardwareinterface as HWI

### Define hardware interrupt 
#Hardware setup: Button between GPIO board pin 16 and ground. 
#Hardware setup: Button between GPIO board pin 40 and ground. 

GPIO.setmode(GPIO.BCM) #BCM naming convention instead of GPIO.board
#variable names based on BOARD convention. pin number uses BCM
gpio16 = 23
GPIO.setup(gpio16, GPIO.IN, pull_up_down = GPIO.PUD_UP)

gpio40 = 21
GPIO.setup(gpio40, GPIO.IN, pull_up_down = GPIO.PUD_UP)

def interrupt_handler(channel):
    if (channel == gpio16):
        print("Interrupt exception: ", channel)
        global CurrentState
        CurrentState.terminator = 1
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
    print("Event Detection loaded. (Interrupts)")
loadeventdetection()

### Define ADC interface 
adc = Adafruit_ADS1x15.ADS1115()
gain = 2.0/3.0 # gain factor for board, 2/3 can read up to 6V, 1 can read up to 4.096V


### Define ultrasonic sensor interface
serialPort = "/dev/ttyAMA0"
maxRange = 5000  # change for 5m vs 10m sensor
sleepTime = 0.01
minMM = 9999
maxMM = 0

### Create the File Name 
d=datetime.datetime.now()
path= os.getcwd()+"/data/"
filename= "data_"+str(d.month)+"_"+str(d.day)+"_"+str(d.hour)+"_"+str(d.minute)+".csv"
#open the file
datafile = open(path+filename,"w+")
datafile.write("Hydrofly Data,Version 0,"+ str(d.month)+"/"+ str(d.day) + "/" +str(d.year) +"\n")
datafile.write("Pressure 0,Pressure 1,Distance\n")



CurrentState = FC.HydroflyState(serialPort)
PreviousState = FC.HydroflyState(serialPort)
PreviousState = copy.deepcopy(CurrentState)

TheVehicle = FC.HydroflyVehicle()
SwitchArmed = 0
Armed = 0

print("Out of Initialization Phase")
sleep(0.5)


while(TheVehicle.FlightMode == 0):
    #CurrentState.updateState(PreviousState, TheVehicle.FlightMode, adc, gain, flag, serialPort)
    print("Mode: ", TheVehicle.FlightMode,"P0:", CurrentState.pressure[0], "P1:", CurrentState.pressure[1], "Dist:", CurrentState.position[2], "VelZ: ", CurrentState.velocity[2])
    PreviousState = copy.deepcopy(CurrentState)

    Armed = np.prod(TheVehicle.ArmCheck(SwitchArmed, SwitchArmed, SwitchArmed, SwitchArmed))
    print(Armed)
    print(TheVehicle.Conditions)
    if (Armed == 1):
        TheVehicle.FlightMode = 1
        TheVehicle.ModeController(CurrentState) #how about a software interrupt that calls this function anytime FlightMode changes/is set


running = True

### Create threads
UpdateState_t1 = threading.Thread(target=CurrentState.updateState, args=(PreviousState, TheVehicle.FlightMode, adc, gain, serialPort, datafile))
CheckState_t2 = threading.Thread(target=CurrentState.CheckState, args=(TheVehicle,))

UpdateState_t1.start()
CheckState_t2.start()

while(CurrentState.terminator ==0):
    print("Tertiary thread exists")
    sleep(0.2)




datafile.close()

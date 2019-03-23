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

### Define hardware interrupt 
#Hardware setup: Button between pin 23 and ground. 
pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

pin2 = 21
GPIO.setup(pin2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#create the file name
d=datetime.datetime.now()
path= os.getcwd()+"/data/"
filename= "data_"+str(d.month)+"_"+str(d.day)+"_"+str(d.hour)+"_"+str(d.minute)+".csv"
#open the file
datafile = open(path+filename,"w+")
datafile.write("Hydrofly Data,Version 0,"+ str(d.month)+"/"+ str(d.day) + "/" +str(d.year) +"\n")
datafile.write("Pressure 0,Pressure 1,Distance\n")

### Define ADC interface 

adc = Adafruit_ADS1x15.ADS1115()

gain = 2.0/3.0 # gain factor for board, 2/3 can read up to 6V, 1 can read up to 4.096V
### Define ultrasonic sensor interface

serialPort = "/dev/ttyAMA0"
maxRange = 5000  # change for 5m vs 10m sensor
sleepTime = 0.01
minMM = 9999
maxMM = 0

def interrupt_handler(channel):
    if (channel == pin):
        print("Interrupt exception", pin)
        global running
        running = False
    elif (channel == pin2):
        print("Interrupt exception", pin2)
        global SwitchArmed
        if (SwitchArmed == 1):
            interrupt_handler(pin) #software interrupt? Not exactly but can be powerful
        else:
            SwitchArmed = 1


GPIO.add_event_detect(pin, GPIO.RISING, callback=interrupt_handler, bouncetime=200)
GPIO.add_event_detect(pin2, GPIO.RISING, callback=interrupt_handler, bouncetime=200)
#use "threading module" for what I call "software interrupts"?

CurrentState = FC.HydroflyState()
CurrentState.initialization(serialPort)

PreviousState = FC.HydroflyState()
PreviousState = copy.deepcopy(CurrentState)

#Loop

##need while true loop for calibration phase
# once all sensors checked and armed, go out of loop
# New CSV file?

TheVehicle = FC.HydroflyVehicle()
flightmode = TheVehicle.FlightMode
flag =0
SwitchArmed = 0
Armed = 0
while(Armed == 0):
    #CurrentState.updateState(PreviousState, flightmode, adc, gain, flag, serialPort)
    print("Mode: ", TheVehicle.FlightMode,"P0:", CurrentState.pressure[0], "P1:", CurrentState.pressure[1], "Dist:", CurrentState.orientation[2], "VelZ: ", CurrentState.velocity[2])
    PreviousState = copy.deepcopy(CurrentState)

    Armed = np.prod(TheVehicle.ArmCheck(SwitchArmed, SwitchArmed, SwitchArmed, SwitchArmed))
    print(Armed)
    print(TheVehicle.Conditions)
    if (Armed == 1):
        TheVehicle.FlightMode = 1


print("Out of Initialization Phase")
sleep(0.5)

running = True
flightmode = TheVehicle.FlightMode
flag = 0
while (running == True):
    CurrentState.updateState(PreviousState, flightmode, adc, gain, flag, serialPort)

    print("pressure 0:", CurrentState.pressure[0], "Pressure 1:", CurrentState.pressure[1], "distance:", CurrentState.orientation[2], "VelocityZ: ", CurrentState.velocity[2], "dt: ")
    #datafile.write(str(CurrentState.pressure[0]) + "," + str(CurrentState.pressure[1]) + "," + str(CurrentState.orientation[2])+ "\n")
    PreviousState = copy.deepcopy(CurrentState)
    sleep(0.01)

datafile.close()

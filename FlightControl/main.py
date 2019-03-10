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

### Define hardware interrupt 
#Hardware setup: Button between pin 23 and ground. 
#only some pins work. Hardware limitation.
pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


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
    print("Interrupt exception")
    global running
    running = False

GPIO.add_event_detect(pin, GPIO.RISING, callback=interrupt_handler, bouncetime=200)

def checkArmed():
    #check terminator variable
    #check wireless virtual switch
    #check arming switch status
    # if all 3 GO
        # turn armed LED on
        # return terminator
    # else 
        # turn LED off 
        # loop until GO condition
    
    return 0

class HydroflyState:
    'Class to hold our state'

    def __init__(self):
        self.FlightMode = 0 
        self.pressure = [0, 0, 0] # p1, p2, p3
        self.theTime = time.time()
        self.currentTime = 0
        self.terminator = 0
        self.velocity = [0, 0, 0] # x, y, z
        self.position = [0, 0, 0] # x, y, z
        self.orientation = [0, 0, 0] # phi, theta, psi, (roll, pitch, yaw?)
        # initialize state here 

    def updateState(self, PreviousState, flightmode, adc, gain, flag, serialPort):
        self.theTime = time.time()
        dt = PreviousState.theTime - self.theTime
        self.FlightMode = flightmode

        self.pressure[0] = utils.voltToPressure(utils.valToVolt(adc.read_adc(0, gain), gain))
        self.pressure[1] = utils.voltToPressure(utils.valToVolt(adc.read_adc(1, gain), gain))
        self.pressure[2] = utils.voltToPressure(utils.valToVolt(adc.read_adc(2, gain), gain))
        self.terminator = flag
        self.orientation[0] = 0.0
        self.orientation[1] = 0.0
        self.orientation[2] = maxSonarTTY.measure(serialPort)
        self.velocity[0] = 0.0
        self.velocity[1] = 0.0
        self.velocity[2] = ((self.orientation[2] - PreviousState.orientation[2])/dt)
 

def initializeInterfaces():
    return 0 


class ModeController:
    'Class to hold our Mode Controller'

    def __init__(varx, vary):
        self.varx = 1
        self.vary = 2
        # initialize state here 


def modeController():
    return 0

def sessionTerminator():
    # I'll be back
    return 0


CurrentState = HydroflyState()
PreviousState = HydroflyState()
PreviousState = copy.deepcopy(CurrentState)

running = True
#Loop

flightmode = 1
flag = 0

##need while true loop for calibration phase
# once all sensors checked and armed, go out of loop
# New CSV file?


while (running == True):

    CurrentState.updateState(PreviousState, flightmode, adc, gain, flag, serialPort)


    print("pressure 0:", CurrentState.pressure[0], "Pressure 1:", CurrentState.pressure[1], "distance:", CurrentState.orientation[2], "VelocityZ: ", CurrentState.velocity[2], "dt: ")
    #datafile.write(str(CurrentState.pressure[0]) + "," + str(CurrentState.pressure[1]) + "," + str(CurrentState.orientation[2])+ "\n")
    PreviousState = copy.deepcopy(CurrentState)
    sleep(0.01)


datafile.close()

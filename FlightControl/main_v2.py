#!/usr/bin/python3
#unfinished for now

import time
import Adafruit_ADS1x15 
import csv
#import board
from time import sleep
import maxSonarTTY
import RPi.GPIO as GPIO


### Define hardware interrupt 
#Hardware setup: Button between pin 23 and ground. 
#only some pins work. Hardware limitation.
pin = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)


### Define ADC interface 

adc = Adafruit_ADS1x15.ADS1115()
gain = 2/3 # gain factor for board, 2/3 can read up to 6V, 1 can read up to 4.096V


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


def voltToPressure(voltage):
    """ converts voltage from ADC to PSIA value"""
    maxPress = 750.0
    minPress = 10.0
    vSource = 5.0 #voltage source to ADC
    vSupplyPress = 5.0 #voltage supply to pressure sensor

    psi = (maxPress-minPress)*voltage/.8/vSource - (maxPress-minPress)/8 + minPress
    return psi

def valToVolt(value, gain):
    """ converts value returned from ADC into an actual voltage depending on gain setting"""
    maxNum = 32768
    v = ((4.096/gain) / float(maxNum)) * value
    return v


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
        self.time = 0
        self.terminator = 0
        self.velocity = [0, 0, 0] # x, y, z
        self.position = [0, 0, 0] # x, y, z
        self.orientation = [0, 0, 0] # phi, theta, psi, (roll, pitch, yaw?)
        # initialize state here 

def initializeInterfaces():
    return 0 


class ModeController:
    'Class to hold our Mode Controller'

    def __init__(varx, vary):
        self.varx = 1
        self.vary = 2
        # initialize state here 


def updateState():
    return 0

def modeController():
    return 0

def sessionTerminator():
    # I'll be back
    return 0


state = HydroflyState() 

running = True
#Loop
while (running == True):
    sensorVal = adc.read_adc(1, gain)
    pressure = voltToPressure(valToVolt(sensorVal, gain))
    
    distance = maxSonarTTY.measure(serialPort)

    print("pressure:", pressure, "distance:", distance)
    sleep(0.01)



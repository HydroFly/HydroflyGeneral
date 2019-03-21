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


# main: Initilize everything
    # utilities: ADC, ultrasonic, etc THOMAS
    # logdata: calls function/process to log data CHLOE
    # updatevalues: THOMAS
        # updateState: calls functions to update values THOMAS
    # processvalues: THOMAS/ADAM
        # ModeController: based on these (& tracking state), sets flight modes and performs differently ADAM/THOMAS
        # AbortController: decides redline conditions THOMAS
            # makes call to abort if needed - pass value back to ModeController THOMAS
    # predictvalues: calls function to predict values based on "phyzakks" because we ain't gonna fly THOMAS/ADAM
    # setsystem: calls functions to set solenoid ADAM
    # shutoff: calls function to shutoff system & finalize logging? ADAM
# end WE PARTY:Q



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

CurrentState = FC.HydroflyState()
PreviousState = FC.HydroflyState()
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

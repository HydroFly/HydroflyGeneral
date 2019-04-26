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

GPIO.setmode(GPIO.BOARD)

gpio27 = 13 #solenoid
GPIO.setup(gpio27, GPIO.OUT, initial=GPIO.LOW)

#gpio27 = 13 #READY LED
#GPIO.setup(gpio27, GPIO.OUT, initial=GPIO.LOW)

gpio23 = 16 #softkill
GPIO.setup(gpio23, GPIO.IN, pull_up_down = GPIO.PUD_UP)

gpio21 = 40 #launch then softkill
GPIO.setup(gpio21, GPIO.IN, pull_up_down = GPIO.PUD_UP)

gpio22 = 15 #launch then softkill for relay
GPIO.setup(gpio22, GPIO.IN, pull_up_down = GPIO.PUD_UP)



def interrupt_handler(channel):
#put all of the check before if statement?
    print("IH: Interrupt detected on pin: ", channel)
    time.sleep(0.05)
    if GPIO.input(channel) != GPIO.LOW:
        return
    else:
        if (channel == gpio23):
            print("Interrupt exception: ", channel)
            global CurrentState
            CurrentState.terminator[0] = 1
            CurrentState.terminator[1] = 1
        elif (channel == gpio21):
            print("Interrupt exception: ", channel)
            global SwitchArmed
            if(SwitchArmed ==1):
                CurrentState.terminator[0] = 1
                CurrentState.terminator[1] = 1
            else:
                SwitchArmed = 1
        elif (channel == gpio22):
            print("Interrupt exception: ", channel)
            global SwitchArmed
            if(SwitchArmed ==1):
                #figure out softkill
                pass
                #CurrentState.terminator[0] = 1
                #CurrentState.terminator[1] = 1
            else:
                SwitchArmed = 1


#load GPIO event detections
def loadeventdetection():
    GPIO.add_event_detect(gpio23, GPIO.FALLING, callback=interrupt_handler, bouncetime=20)
    GPIO.add_event_detect(gpio21, GPIO.FALLING, callback=interrupt_handler, bouncetime=20)
    GPIO.add_event_detect(gpio22, GPIO.FALLING, callback=interrupt_handler, bouncetime=20)

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
#path= os.getcwd()+"/data/"
path="/home/pi/Hydrofly/FlightControl/data/"
filename= "data_"+str(d.month)+"_"+str(d.day)+"_"+str(d.hour)+"_"+str(d.minute)+".csv"
#open the file
datafile = open(path+filename,"w+")
datafile.write("Hydrofly Data,Version 1,"+ str(d.month)+"/"+ str(d.day) + "/" +str(d.year) +"\n")
datafile.write("Pressure 0,Pressure 1,Pressure 2,Distance\n")
print("Log File Created")

CurrentState = FC.HydroflyState(serialPort, adc, gain, datafile)
TheVehicle = FC.HydroflyVehicle(datafile) 
print("State and Vehicle Objects Created")

#initiate briefcase monitoring
SwitchArmed = 0
Armed = 0

#
#GPIO.output(gpio27, CurrentState.solenoid_state)
#sleep(0.5)
#GPIO.output(gpio27, CurrentState.solenoid_state)
print("Out of Initialization Phase")
sleep(0.5)


### Create threads ###
#Starts State update and redline condition checking
UpdateState_t1 = threading.Thread(target=CurrentState.update_state, args=(adc, gain, serialPort, TheVehicle))
check_state_t2 = threading.Thread(target=CurrentState.check_state, args=(TheVehicle,))
print("State Updater and Checker Threads Created")
sleep(0.5)

UpdateState_t1.start()
check_state_t2.start()
print("Threads Started")
sleep(0.5)

"""
if (TheVehicle.testmode == 3):
    ref_time = time.time()
    while(time.time() < (ref_time + 3)):
        TheVehicle.solenoid_state = 1
        GPIO.output(gpio27, CurrentState.solenoid_state)
        print("MN: FlightMode: ", TheVehicle.flight_mode, "Height: ", CurrentState.position[2], "solenoid_state:", CurrentState.solenoid_state )

    print("OUT OF LOOP")
    TheVehicle.abort(CurrentState)
    GPIO.output(gpio27, CurrentState.solenoid_state)
    while (True):
        if (sum(CurrentState.terminator) == 2):
            sleep(0.5) #let all threads finish
            datafile.close()
            break

"""

while(TheVehicle.flight_mode == 0):
    print("System Currently Unarmed. Press Button to Arm when Ready.", CurrentState.terminator)
    if (SwitchArmed == 1):
        print("System Armed")
        TheVehicle.Conditions = [1,1,1,1] #once sensors added, just sent one of the elements
        TheVehicle.mode_controller(1) #how about a software interrupt that calls this function anytime flight_mode changes/is set
    sleep(.2)

#Ground Control Arming, aka, launch!! # <talk to chloe about arming condition
#if statement on conditions being all 1, then mode_controller(1)


#Hardware Commands / Flight Mission
while(CurrentState.terminator[0] == 0 and TheVehicle.flight_mode == 1):

    TheVehicle.run(CurrentState)
    TheVehicle.control(CurrentState)
    GPIO.output(gpio27, CurrentState.solenoid_state)
    print("MN: FlightMode: ", TheVehicle.flight_mode, "Height: ", CurrentState.position[2], "solenoid_state:", CurrentState.solenoid_state )
    if (CurrentState.position[2] >= 2 and TheVehicle.hover_endtime == 0):
        TheVehicle.mode_controller(2)

while(CurrentState.terminator[0] == 0 and TheVehicle.flight_mode == 2):
    TheVehicle.run(CurrentState)
    TheVehicle.control(CurrentState)
    GPIO.output(gpio27, CurrentState.solenoid_state)
    print("MN: FlightMode: ", TheVehicle.flight_mode, "Height: ", CurrentState.position[2], "solenoid_state:", CurrentState.solenoid_state )
    if (time.time() >= TheVehicle.hover_endtime):
        TheVehicle.mode_controller(3)

print("MN: Hovered for 3 seconds!")
while(CurrentState.terminator[0] == 0 and TheVehicle.flight_mode == 3):
    
    TheVehicle.run(CurrentState)
    TheVehicle.control(CurrentState)
    GPIO.output(gpio27, CurrentState.solenoid_state)
    print("MN: FlightMode: ", TheVehicle.flight_mode, "Height: ", CurrentState.position[2], "solenoid_state:", CurrentState.solenoid_state )


TheVehicle.abort(CurrentState)
GPIO.output(gpio27, CurrentState.solenoid_state)
while (True):
    if (sum(CurrentState.terminator) == 2):
        sleep(0.5) #let all threads finish
        datafile.close()
        break
exit()

#!/usr/bin/python3
#unfinished for now

import time
import Adafruit_ADS1x15 
import csv
import board
from time import sleep
import maxSonarTTY
import RPi.GPIO as GPIO

### Define hardware interrupt 
#Hardware setup: Button between pin 23 and ground. 
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


running = True
#Loop
while (running == True):
    sensorVal = adc.read_adc(1, gain)
    pressure = voltToPressure(valToVolt(sensorVal, gain))
    
    distance = maxSonarTTY.measure(serialPort)

    print("pressure:", pressure, "distance:", distance)

    sleep(0.01)



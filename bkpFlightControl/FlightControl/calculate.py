#!/usr/bin/python3
#unfinished for now

import time
#import maxSonarTTY
import Adafruit_ADS1x15 
import adafruit_ads1x15.ads1115 as ADS
#import GPIO
import csv
import board
import busio
from adafruit_ads1x15.analog_in import AnalogIn

#we will need to change the gain to 2/3 and find the factor to multiply the result by
#we can compare with the adafruit stuff that returns a voltage
#reference https://cdn-learn.adafruit.com/downloads/pdf/adafruit-4-channel-adc-breakouts.pdf?timestamp=1551486012

#Define all interfaces
adc = Adafruit_ADS1x15.ADS1115() #may want to customize interface here

serialPort = "dev/ttyAMA0"
maxPress = 750.0
minPress = 10.0
vSource = 5.0 #voltage source to the adafruit thingy
vSupplyPress = 5.0 #voltage supply to pressure sensor
pressure = 0.0
temp = 0.0
vOut = 0.0 #calculated voltage that the pressure sensor returns
maxNum = 32768 #upper end of what the adafruit thingy can return
gain = 2/3

#Loop
while True:
    values = [0]*4
    for i in range(4):
        values[i] = adc.read_adc(i, gain)

    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
    
    #this calculates the pressure from the number the pressure sensor returns
    vOut = ((4.096/gain) / float(maxNum)) * float(adc.read_adc(1, gain))
    

    pressure = (maxPress-minPress)*vOut/.8/vSource - (maxPress-minPress)/8 + minPress
    print("pressure:", pressure)
    print("vOut: ",vOut)
    #print("vSupplyPress: ", vSupplyPress)
    #print("maxNum: ", maxNum)
    #print("float value: ",float(values[2]))
    #print("maxPress: ",maxPress)
    #print("minPress: ",minPress)
    #print("vSource: ",vSource)


#    print(values[0])
    
                
    time.sleep(0.5)


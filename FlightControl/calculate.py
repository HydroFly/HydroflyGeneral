#!/usr/bin/python3
#unfinished for now

import time
#import maxSonarTTY
import Adafruit_ADS1x15
#import GPIO
import csv

#Define all interfaces
adc = Adafruit_ADS1x15.ADS1115() #may want to customize interface here

serialPort = "dev/ttyAMA0"
maxPress = 750.0
minPress = 0.0
vSource = 5.0 #voltage source to the adafruit thingy
vSupplyPress = 5.0 #voltage supply to pressure sensor
pressure = 0.0
temp = 0.0
vOut = 0.0 #calculated voltage that the pressure sensor returns
maxNum = 32768 #upper end of what the adafruit thingy can return

#Loop
while True:
    values = [0]*4
    for i in range(4):
     values[i] = adc.read_adc(i, gain=1)

    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
    
    #this calculates the pressure from the number the pressure sensor returns
    vOut = (vSource / float(maxNum)) * adc.read_adc(0, gain=1)
    
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


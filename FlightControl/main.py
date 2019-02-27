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


#Loop
while True:
    values = [0]*4
    for i in range(4):
        values[i] = adc.read_adc(i, gain=1)

    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))

    with open('testrecord.csv', 'wb') as csvfile:
        writerr = csv.writer(csvfile, delimiter=' ', 
                quotechar='|', quoting=csv.QUOTE_MINIMAL)
        writerr.writerow(values)
                 
                
    time.sleep(0.5)

    print("looped")


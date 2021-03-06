#libraries
import time
import os
import datetime
import Adafruit_ADS1x15
import curses
import RPi.GPIO as GPIO

#define variables
adc = Adafruit_ADS1x15.ADS1115()
d=datetime.datetime.now()
path= os.getcwd()+"/data/"
filename= "data_"+str(d.month)+"_"+str(d.day)+"_"+str(d.hour)+"_"+str(d.minute)+".csv"
GAIN = 1

#set stuff for curses
stdscr = curses.initscr()
curses.noecho()
curses.cbreak()
stdscr.nodelay(1)

#set up GPIO pin 40
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


stdscr.addstr(0,23,"getting data, presss a key to exit")

#open file
datafile = open(path+filename,"w+")

#return data/put in file
print('Reading ADS1x15 values into the file, press q to stop...')
datafile.write('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |\n'.format(*range(4)))
datafile.write('-' * 37+"\n")
while True:
    values = [0]*4
    for i in range(4):
        values[i] = adc.read_adc(i, gain=GAIN)
    datafile.write('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |\n'.format(*values))
    time.sleep(0.5)
    #get userinput?
    stdscr.refresh()
    c = stdscr.getch()
    if c != curses.ERR:
        break
    if GPIO.input(40) == GPIO.HIGH:
        break


#close the file
datafile.close()
os.system('reset')

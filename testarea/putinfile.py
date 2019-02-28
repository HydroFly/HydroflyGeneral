#import libraries
import datetime
import os

d=datetime.datetime.now()
path= os.getcwd()
filename= "data_"+str(d.month)+"_"+str(d.day)+"_"+str(d.hour)+"_"+str(d.minute)+".csv"
path= path+"/data/"
datafile = open(path+filename,"w+")

print(filename)
datafile.write("hello world")

datafile.close()

import socket
import time
import datetime
import os
def send(data, port):
    s = socket.socket()
    s.bind(('', port))
    s.listen(5)
    c, addr = s.accept()
    print('Got connection from ', addr)
    c.send(data)
    c.close()

d=datetime.datetime.now()
path= os.getcwd()+"/data/"
minute = d.minute
hour = d.hour
#I need to use military time
#RUN THIS AFTER FLIGHT CONTROL
while True:
    try:
        filename= "data_"+str(d.month)+"_"+str(d.day)+"_"+str(hour)+"_"+str(minute)+".csv"
        exists = os.path.isfile(path+filename)
        print(filename)
        if exists:
            print("File exists. I am opening it")
             #open the file
            datafile = open(path+filename,"r")
            print("Log File Opened")
            break
        else: 
            if minute != 0:
                minute = minute - 1
            else:
                minute = 59
                hour = hour - 1
            pass
    except FileNotFoundError:
        print("Something went wrong")
        break

datafile.close()

if __name__ == '__main__':
    port = 1025
    num = 1

while True:
    #check terminator data to exit
    print('Hey! Sending data')
    words = 'HelloWorld'
    thedata =  words + str(num)
    data = bytes(thedata, 'utf-8')
    print("Send data: ", data)
    send(data,port)
    port = port + 1
    num = num + 1


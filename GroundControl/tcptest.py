import socket
import sys
def read(port):
    s = socket.socket()
    host = '10.123.180.190' #ip of FC-raspberrypi
    s.connect((host,port))
    try:
        msg = s.receive(1024)
        s.close()
    except socket.error as msg:
        sys.stderr.write('error', msg[1])
        s.close()
        print(close) #was in python2 format print 'close'
        sys.exit(2)
    return msg

if __name__ == '__main__':
    port = 1025
    while True:
        print("Hey, checking TCP socket")
        data = read(port)
        print ('I just read ', data)
        print ('Port number is: ', port)
        port = port + 1

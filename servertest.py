import socket
import time
def send(data, port):
    s = socket.socket()
    s.bind(('', port))
    s.listen(5)
    c, addr = s.accept()
    print('Got connection from ', addr)
    c.send(data)
    c.close()

if __name__ == '__main__':
    port = 1025
    num = 1

while True:
    print('Hey! Sending data')
    words = 'HelloWorld'
    thedata =  words + str(num)
    data = bytes(thedata, 'utf-8')
    print("Send data: ", data)
    send(data,port)
    port = port + 1
    num = num + 1



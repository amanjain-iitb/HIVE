import socket
import time

HOST = '192.168.0.180'              # The server's hostname or IP address (NodeMCU)
PORT = 80                           # The port used by the server

def send(message,sock):
    #b=time.time()
    sock.sendall(message)
    #dealay
    #time.sleep(0.5)
    
    #while (s.recv(1024)!=b"got em'"):
        
        #sock.sendall(message)
    #print(time.time()-b)
    return True
    
    

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)              #attach a socket to port and connect to host
s.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)                #make sure all messages are instantaneously sent
s.connect((HOST, PORT))
while True:
    
    user=input("(E)xit  / pwm+dir  : ")
    a=time.time()
    if len(user)==5:
        send(str.encode(user),s)    #info send as a 5 digit bit string, first four digits for pwm width, last for mode(frwd/bckwd/rght/left)
    elif user=="E":
        break
    else:
        send(b"40400",s)
    print(time.time()-a, " MAIN LOOP TIME")

s.close()                                           #disconnect with NodeMCU

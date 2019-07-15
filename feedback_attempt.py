import numpy as np
import imutils
import cv2
import urllib.request #for reading image from URL
import socket
import time

def send(message,sock):
    a=time.time()
    sock.sendall(message)
    while (sock.recv(1024)!=b"got em'") and (time.time()-a<2):
        pass
    return True

#def find_dir(dist):
    



HOST = '192.168.43.10'  # The server's hostname or IP address
PORT = 80
   
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
#s.connect((HOST, PORT))

final_pos=(30,100)
kernel = np.ones((9,9),np.uint8)
camera = cv2.VideoCapture(0)

direc='_'
distance_list=[]
dist_coeff=1


url='http://192.168.0.137:8080/shot.jpg'

#IP webcam image stream
#URL = 'http://192.168.43.138:8080/shot.jpg'
#urllib.request.urlretrieve(URL, 'shot1.jpg')
while True:
    
    initial=time.time()
    imgResp=urllib.request.urlopen(url)
    imgNp=np.array(bytearray(imgResp.read()),dtype=np.uint8)
    frame=cv2.imdecode(imgNp,-1)
    frame = cv2.imread('shot1.jpg')
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (30,100,10), (70,255,255))
    gradient = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernel)
   
   
        
    cv2.circle(frame,final_pos,30,(255,0,255))
    cv2.imshow("grad", gradient)
    cnts = cv2.findContours(gradient.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    print("*******************************************")

    print(time.time()-initial, "time taken image processing block")
    initial=time.time()
    
    if len(cnts)==0:
        pass
    else:
        i = max(cnts, key=cv2.contourArea)
        
    #        print(cv2.contourArea(i))
            
        if cv2.contourArea(i)<10000 and cv2.contourArea(i)>50:
            M = cv2.moments(i)
            (x,y)=(int(M['m10']/M['m00']),int(M['m01']/M['m00']))
            [vx,vy,_,_] = cv2.fitLine(i, cv2.DIST_L2,0,0.01,0.01)
            
            cv2.circle(frame,(x,y),10,(255,0,255))
            cv2.line(frame,(x,y),(x+100*vx,y+100*vy),(0,105,255),2)
            cv2.line(frame,(x,y),final_pos,(0,105,255),2)
            
            u=np.array([(x-final_pos[0]),(y-final_pos[1])])
            v=np.array([vx[0],vy[0]])
            angle=np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v))
            angle=np.arccos(angle)*90/1.57
            dist=np.linalg.norm(u)
            if angle>90:
                    angle=180-angle
            
        
            print("distance", dist)
            print("angle" ,angle)
            print('direc ',direc)
            print('direction list',distance_list)
            print("******")
           
            if angle>=80:
                if dist>30:
                    print(direc)
                    if direc=='1':
                        dist_time=str(int(dist*dist_coeff))
                        dist_time=dist_time.rjust(4,'0') +'1'
                        send(str.encode(dist_time),s)
                        distance_list.append(dist)
                        print("movong",dist_time)
                    elif direc=='2':
                        dist_time=str(int(dist*dist_coeff))
                        dist_time=dist_time.rjust(4,'0') +'2'
                        send(str.encode(dist_time),s)
                        distance_list.append(dist)
                        print("movong",dist_time)
                    elif direc=='_':
                        if len(distance_list)==0:
                            print("inside trial")
                            distance_list.append(dist)
                            send(str.encode('00251'),s)
                        else:
                            if dist<distance_list[-1]:
                                direc='1'
                            else:
                                direc='2' 
                    else:
                        print("not supposed to be here")
                    
                else:
                    direc='_'
                    distance_list=[]
            else:
                ang=int(1500/angle)
                if ang>100:
                    ang=100
                ang=str(ang)
                ang=ang.rjust(4,'0') +'3'
                send(str.encode("00303"),s)
                distance_list.append(dist)
                print("angling",ang)
            
        else:
            pass
                
                
        cv2.drawContours(frame, [i], -1, (0,255,0), 3)
        cv2.putText(frame,str(dist), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
        print(time.time()-initial, "time taken image processing block")     
            
            
    '''
            [vx,vy,x,y] = cv2.fitLine(i, cv2.DIST_L2,0,0.01,0.01)
            cv2.line(frame,(x,y),(x+100*vx,y+100*vy),(0,105,255),2)
            cv2.line(frame,(x,y),final_pos,(0,105,255),2)
            cv2.circle(frame,(x,y),10,(255,0,255))
            u=np.array([(x[0]-final_pos[0]),(y[0]-final_pos[1])])
            v=np.array([vx[0],vy[0]])
            angle=np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v))
            angle=np.arccos(angle)*90/1.57
            print(angle)
            
            if angle<=5 or angle>=175:
                pass
            else:
                s.sendall(b'07002')
        else:
            pass '''
                
            
                
       
        
    
    cv2.imshow("Frame", frame)
    print("*******************************************")

    if cv2.waitKey(1) & 0xFF ==ord('q'):    
        break
 
# cleanup the camera and close any open windows
s.close()
camera.release()
cv2.destroyAllWindows()


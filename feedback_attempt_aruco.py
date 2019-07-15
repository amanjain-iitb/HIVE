import numpy as np
import imutils
import cv2
import urllib.request 
import socket
import time
import cv2.aruco as aruco
import math

def centre_(c):
    m13=(c[0][1]-c[2][1])/(c[0][0]-c[2][0])
    m24=(c[1][1]-c[3][1])/(c[1][0]-c[3][0])
    if m13-m24==0:
        return((c[0][0]+c[2][0])/2,(c[0][1]+c[2][1])/2)
    x=(c[1][1]-c[2][1]+(m13*c[2][0])-(m24*c[1][0]))/(m13-m24)
    y=(m24*x)+c[1][1]-(m24*c[1][0])
    return int_((x,y))

id_list=[8,]

marker_size=10

def int_(array):
    try :
         ar=(int(array[0]),int(array[1]))
         return ar
    except:
        return array
    
#        ar=
#    if array[0]==float('NaN') or array[1]==float('NaN'):
#        return array
#    return (int(array[0]),int(array[1]))
#    

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
def rotationMatrixToEulerAngles(R) :
    assert(isRotationMatrix(R)) 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

# Get the camera calibration path
calib_path=""
camera_matrix=np.loadtxt(calib_path+'cameraMatrix.txt',delimiter=',')
camera_distortion=np.loadtxt(calib_path+'cameraDistortion.txt',delimiter=',')

# 180 deg rotation around the x axis
R_flip=np.zeros((3,3),dtype=np.float32)
R_flip[0,0]=1.0
R_flip[1,1]=-1.0
R_flip[2,2]=-1.0

aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_50 )
parameters=aruco.DetectorParameters_create()

cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

font=cv2.FONT_HERSHEY_PLAIN
def send(message,sock):
    a=time.time()
    sock.sendall(str.encode(message))
    while (sock.recv(1024)!=b"got em'") and (time.time()-a<2):
        pass
    return True

HOST = '192.168.43.10'  # The server's hostname or IP address
PORT = 80


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
s.connect((HOST, PORT))


final_pos=np.array([30,300])
kernel = np.ones((9,9),np.uint8)

cap = cv2.VideoCapture("http://192.168.43.208:8080/video")

while True:
    ret,frame=cap.read()
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #frame = imutils.resize(frame, width=900)
    cv2.circle(frame,tuple(final_pos),30,(255,0,255))
    
    corners,ids,rejected=aruco.detectMarkers(image=gray,dictionary=aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=camera_distortion)
    
    if corners!= []:
        if ids[0]==8:
            centre=centre_(corners[0][0])
            ret=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec,tvec,_objPoints=ret[0][0,0,:],ret[1][0,0,:],ret[2][0,0,:]
            u=final_pos-np.array(centre)
            angle=math.degrees(np.arccos(u[0]/np.linalg.norm(u)))
            if u[1]>0:
                angle*=-1
            
            aruco.drawDetectedMarkers(frame,corners)
            aruco.drawAxis(frame,camera_matrix,camera_distortion,rvec,tvec,10)
            
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            _, _, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
            dist=np.linalg.norm(u)
           
            if dist<30:
                break
            
            cv2.circle(frame,centre,5,(255,0,205),2)
            anglee=math.degrees(yaw_marker)-angle
            if anglee<0:
                anglee+=360
            if anglee<361 and anglee>0:
                print("Check confirm ", anglee)
            else:
                print("NOOOOOOOOOOOOOOO")
            if anglee=<5: 
                lin='1'
                ang='0'
            elif anglee=<10:
                lin='1'
                ang='4'
            elif anglee=<90:
                lin='10'
                ang='4'
            elif anglee=<170:
                lin='20'
                ang='3'
            elif anglee=<175:
                lin='2'
                ang='3'
            elif anglee=<185:
                lin='2'
                ang='0'
            elif anglee=<190:
                lin='2'
                ang='4'
            elif anglee=<270:
                lin='20'
                ang='4'
            elif anglee=<350:
                lin='10'
                ang='3'
            elif anglee=<355:
                lin='1'
                ang='3'
            else:
                lin='1'
                ang='0'
            if ang[-1]!=0:
                send('0050'+ang,s)
            if lin[-1]!=0:
                send('0100'+lin,s)
                
        
               
              
               
            
            
            cv2.line(frame,centre ,tuple(final_pos),(0,105,255),2)
            cv2.putText(frame,'distance:'+str(dist), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),2)
                   
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF ==ord('q'):    
        break
 
# cleanup the camera and close any open windows
#s.close()
camera.release()
cv2.destroyAllWindows()
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
import numpy as np
from itertools import permutations 
import cv2
import socket
import cv2.aruco as aruco
import math

#**************************************************************************************************************************************

def send(message,sock):
#    a=time.time()
    sock.sendall(str.encode(message))
#    while (sock.recv(1024)!=b"got em'") and ((time.time()-a)<0.10):
#        pass
    return True

def centre_(c):
    '''
    if c[0][0]==c[2][0] or c[1][0]==c[3][0]:
        return int_(((c[0][0]+c[2][0])/2,(c[0][1]+c[2][1])/2))
    m13=(c[0][1]-c[2][1])/(c[0][0]-c[2][0])
    m24=(c[1][1]-c[3][1])/(c[1][0]-c[3][0])
    if m13==m24:
        return int_((c[0][0]+c[2][0])/2,(c[0][1]+c[2][1])/2)
    x=(c[1][1]-c[2][1]+(m13*c[2][0])-(m24*c[1][0]))/(m13-m24)
    y=(m24*x)+c[1][1]-(m24*c[1][0])
    return int_((x,y))
    '''
    return int_(((c[0][0]+c[1][0]+c[2][0]+c[3][0])/4,(c[0][1]+c[1][1]+c[2][1]+c[3][1])/4))

def set_to_none(dict_):
    for key in dict_:
        dict_[key]=None
        
def int_(array):
    try :
         ar=(int(array[0]),int(array[1]))
         return ar
    except:
        return array
   

def decide_pos():
    for key in current_pos:
        if current_pos[key]==None:
            return False
    perm=permutations(shape_pos)
    min_dist=dist_(shape_pos,list(current_pos.values()))
    min_i=shape_pos
    for i in perm:
        if dist_(i,list(current_pos.values()))<min_dist:
            min_i=i
    j=0
    for key in final_pos:
        final_pos[key]=min_i[j]
        j+=1
    return True
        
def dist_(list1,list2):
    dist=0
    for i in range(no_bots):
        dist+=np.linalg.norm(np.array(list1)-np.array(list2))
    return dist

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

def shape_maker(HEIGHT,WIDTH,no_bots,shape):
    if no_bots==1:
        shape_pos=(int_(WIDTH/2,HEIGHT/2))
    elif no_bots==2:
        shape_pos=(int_(WIDTH/5,HEIGHT/2),int_(4*WIDTH/5,HEIGHT/2))
    elif no_bots==3:
        if shape=='r_tri':
            shape_pos=(int_(WIDTH/5,HEIGHT/6),int_(WIDTH/5,5*HEIGHT/6),int_(3*WIDTH/5,HEIGHT/6))
        elif shape=='e_tri':
            shape_pos=(int_((WIDTH-HEIGHT/3**0.5)/2,3*HEIGHT/4),int_((WIDTH+HEIGHT/3**0.5)/2,3*HEIGHT/4),int_(5*WIDTH/6,HEIGHT/4))
        else:
            shape_pos=(int_(WIDTH/6,HEIGHT/2),int_(3*WIDTH/6,HEIGHT/2),int_(5*WIDTH/6,HEIGHT/2))
    elif no_bots==4:
        if shape=='square':
            pass
        
            
    
    
    
#**********************************************************************************************************************************

# Get the camera calibration path
calib_path=""
camera_matrix=np.loadtxt(calib_path+'cameraMatrix.txt',delimiter=',')
camera_distortion=np.loadtxt(calib_path+'cameraDistortion.txt',delimiter=',')

# 180 deg rotation around the x axis
R_flip=np.zeros((3,3),dtype=np.float32)
R_flip[0,0]=1.0
R_flip[1,1]=-1.0
R_flip[2,2]=-1.0

aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_6X6_100 )
parameters=aruco.DetectorParameters_create()

font=cv2.FONT_HERSHEY_PLAIN
id_list=[1,2]
marker_size=10
no_bots=2


HOST_LIST={1:'192.168.43.3',2:'192.168.43.37'}                                    #HOST1,HOST2,........
PORT = 80

s={}
for key in id_list:
    sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
    sock.connect((HOST_LIST[key], PORT))
    s[key]=sock


final_pos={1:None,2:None}
current_pos={1:None,2:None}


WIDTH=1280
HEIGHT=720
cap= WebcamVideoStream(src="http://192.168.43.49:8080/video").start()
#cap= WebcamVideoStream(src=0).start()
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)

lin=''
ang=''

shape_pos=[]
shape_maker(HEIGHT,WIDTH,no_bots)
#shape_pos=((30,300),(650,300))
shape=False

while True:
    ret,frame=cap.read()
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    gray=cv2.bilateralFilter(gray,5,75,75)

    corners,ids,rejected=aruco.detectMarkers(image=gray,dictionary=aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=camera_distortion)
    
    if corners!= []:
        for i in id_list:
            if i in ids:
                
                centre=centre_(corners[list(ids).index(i)][0])
                current_pos[i]=centre
                print (current_pos,final_pos,shape_pos)
                
                shape=decide_pos()
                if shape:
                    cv2.circle(frame,tuple(final_pos[i]),30,(255,0,255),4)
                    ret=aruco.estimatePoseSingleMarkers([corners[list(ids).index(i)],], marker_size, camera_matrix, camera_distortion)
                    rvec,tvec,_objPoints=ret[0][0,0,:],ret[1][0,0,:],ret[2][0,0,:]
                    u=final_pos[i]-np.array(centre)
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
                        print('done')
                        break
                    
                    cv2.circle(frame,centre,5,(255,0,205),2)
                    anglee=math.degrees(yaw_marker)-angle
                    if anglee<0:
                        anglee+=360
                    '''    
                    if anglee<361 and anglee>=0:
                        print("Check confirm ", anglee)
                    else:
                        print("NOOOOOOOOOOOOOOO")
                    '''
                    if anglee<=5: 
                        lin='1'
                        ang='0'
                    elif anglee<=10:
                        lin='1'
                        ang='4'
                    elif anglee<=90:
                        lin='10'
                        ang='4'
                    elif anglee<=170:
                        lin='20'
                        ang='3'
                    elif anglee<=175:
                        lin='2'
                        ang='3'
                    elif anglee<=185:
                        lin='2'
                        ang='0'
                    elif anglee<=190:
                        lin='2'
                        ang='4'
                    elif anglee<=270:
                        lin='20'
                        ang='4'
                    elif anglee<=350:
                        lin='10'
                        ang='3'
                    elif anglee<=355:
                        lin='1'
                        ang='3'
                    else:
                        lin='1'
                        ang='0'
                       
                    if ang[-1]!='0':
                        ang_coeff='0010'
                        send(ang_coeff+ang,s[i])
                    if lin[-1]!='0':
                        if dist<20:
                            send('0005'+lin,s[i])
                        else:
                            send('0020'+lin,s[i])
                 
                    cv2.line(frame,centre ,tuple(final_pos[i]),(0,105,255),2)
                    cv2.putText(frame,'distance:'+str(dist), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),2)
                else:
                    pass
    else:
        set_to_none(current_pos)
        shape=False
        lin=''
        ang=''
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF ==ord('q'):    
        break
 
# cleanup the camera and close any open windows

cap.release()
cv2.destroyAllWindows()
for key in s:
    s[key].close()    

        
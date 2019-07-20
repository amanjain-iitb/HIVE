import numpy as np
import cv2
import socket
import cv2.aruco as aruco
import math

#**************************************************************************************************************************************

def send(message,sock):
    sock.sendall(str.encode(message))
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

aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_50 )
parameters=aruco.DetectorParameters_create()

font=cv2.FONT_HERSHEY_PLAIN
id_=0
marker_size=10



HOST='192.168.43.147'                              
PORT = 80
sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
sock.connect((HOST, PORT))

WIDTH=1280
HEIGHT=720

#path maker
pos_list=[]
drawing = False # true if mouse is pressed
ix,iy = -1,-1
# mouse callback function
def draw_circle(event,x,y,flags,param):
    global ix,iy,drawing,mode
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix,iy = x,y
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == True:
            pos_list.append((x,y))
            cv2.circle(img,(x,y),5,(0,0,255),-1)
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.circle(img,(x,y),5,(0,0,255),-1)
img = np.zeros((HEIGHT,WIDTH,3), np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image',draw_circle)
while(1):
    cv2.imshow('image',img)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break
cv2.destroyAllWindows()


#pos_list=[int_((30,HEIGHT/2)),int_((200,HEIGHT/2)),int_((500,HEIGHT/2)),int_((500,HEIGHT/2-60)),int_((100,HEIGHT/2-60)),int_((30,HEIGHT/2-60))]
#pos_list=[(588, 126), (581, 122), (569, 118), (560, 115), (550, 117), (544, 117), (540, 118), (532, 118), (525, 118), (513, 115), (503, 113), (492, 110), (484, 109), (476, 108), (467, 107), (457, 106), (452, 104), (442, 103), (434, 103), (428, 102), (423, 102), (419, 102), (410, 100), (406, 100), (401, 100), (397, 100), (392, 100), (388, 100), (384, 101), (377, 102), (375, 102), (369, 102), (365, 102), (356, 102), (348, 102), (340, 102), (332, 102), (328, 102), (322, 102), (318, 102), (314, 102), (306, 103), (302, 105), (293, 106), (285, 109), (280, 111), (271, 117), (265, 118), (261, 122), (252, 127), (245, 132), (237, 138), (230, 142), (224, 146), (218, 149), (211, 151), (208, 155), (200, 158), (195, 160), (191, 162), (183, 163), (180, 165), (175, 167), (168, 174), (164, 178), (159, 183), (154, 191), (152, 194), (148, 199), (147, 202), (146, 202), (144, 206), (143, 209), (143, 212), (139, 218), (137, 237), (137, 246), (135, 266), (134, 272), (133, 282), (132, 289), (130, 298), (128, 312), (128, 321), (129, 326), (129, 330), (130, 330), (131, 334), (131, 335), (132, 335), (132, 338), (135, 341), (137, 342), (139, 345), (141, 350), (143, 353), (147, 359), (148, 367), (150, 369), (152, 371), (154, 375), (155, 376), (157, 378), (160, 379), (163, 382), (167, 384), (172, 386), (176, 388), (180, 390), (183, 392), (188, 394), (189, 396), (192, 399), (193, 401), (198, 402), (198, 403), (201, 406), (206, 410), (208, 411), (214, 417), (220, 420), (223, 422), (230, 426), (237, 429), (242, 430), (247, 430), (253, 434), (256, 435), (261, 437), (271, 439), (279, 443), (291, 447), (296, 451), (308, 460), (315, 467), (336, 479), (349, 486), (359, 492), (367, 494), (373, 498), (378, 502), (384, 505), (388, 509), (391, 511), (397, 514), (401, 518), (405, 519), (408, 520), (408, 521), (410, 522), (429, 521), (436, 520), (452, 519), (463, 519), (474, 519), (487, 520), (501, 520), (512, 522), (520, 523), (524, 525), (528, 527), (535, 530), (542, 533), (554, 539), (558, 541), (568, 545), (572, 548), (580, 552), (584, 554), (586, 558), (593, 562), (596, 566), (603, 570), (610, 575), (612, 578), (618, 580), (621, 583), (624, 586), (629, 593), (631, 595), (636, 603), (637, 609), (640, 614), (641, 622), (642, 622), (644, 626), (645, 631), (647, 635), (647, 643), (648, 656), (648, 665), (648, 670), (648, 674), (648, 675), (648, 678), (648, 678), (648, 682), (647, 685), (646, 689), (643, 694), (642, 698), (640, 704), (637, 711), (634, 714), (632, 717), (630, 718), (625, 722), (620, 725), (616, 726), (612, 729), (605, 732), (598, 734), (595, 736), (592, 740), (588, 743), (580, 750), (575, 754), (567, 761), (558, 765), (552, 769), (546, 770), (538, 774), (530, 774), (517, 777), (501, 779), (480, 779), (463, 778), (441, 776), (422, 776), (406, 776), (395, 776), (387, 776), (380, 776), (372, 775), (368, 774), (362, 772), (355, 771), (351, 771), (344, 770), (340, 770), (334, 770), (328, 770), (311, 772), (300, 774), (282, 779), (270, 783), (260, 788), (250, 792), (247, 794), (240, 797), (232, 798), (223, 801), (218, 801), (208, 801), (200, 801), (193, 801), (189, 799), (181, 798), (177, 798), (173, 798), (168, 796), (163, 794), (158, 794), (152, 792), (145, 790), (140, 789), (137, 788), (131, 787), (128, 786), (124, 786), (121, 785), (118, 784), (116, 783), (113, 783), (112, 782), (107, 780), (107, 779), (104, 778), (101, 777)]

cap = cv2.VideoCapture("http://192.168.43.49:8080/video")
#cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,HEIGHT)
kernel = np.ones((9,9),np.uint8)


lin=''
ang=''

go_for=0

while True:
    ret,frame=cap.read()
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #gray=cv2.bilateralFilter(gray,5,75,75)

    corners,ids,rejected=aruco.detectMarkers(image=gray,dictionary=aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=camera_distortion)
    
    if corners!= []:
        if id_ ==ids[0]: 
            centre=centre_(corners[0][0])
        
          
            cv2.circle(frame,pos_list[go_for],10,(255,0,255),4)
            ret=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec,tvec,_objPoints=ret[0][0,0,:],ret[1][0,0,:],ret[2][0,0,:]
            u=np.array(pos_list[go_for])-np.array(centre)
            angle=math.degrees(np.arccos(u[0]/np.linalg.norm(u)))
            if u[1]>0:
                angle*=-1
            
            aruco.drawDetectedMarkers(frame,corners)
            aruco.drawAxis(frame,camera_matrix,camera_distortion,rvec,tvec,10)
            
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T

            _, _, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
            dist=np.linalg.norm(u)
            #print(dist)
            if dist<10:
                print('done point ',pos_list[go_for])
                go_for+=1
                if go_for==(len(pos_list)-1):
                    break
            
            cv2.circle(frame,centre,5,(255,0,205),2)
            anglee=math.degrees(yaw_marker)-angle
            if anglee<0:
                anglee+=360
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
            '''
            if anglee<=5:
                lin='1'
                ang='0'
            elif anglee<=90:
                lin='0'
                ang='4'
            elif anglee<=175:
                lin='0'
                ang='3'
            elif anglee<=185:
                lin='2'
                ang='0'
            elif anglee<=270:
                lin='0'
                ang='4'
            elif anglee<=355:
                lin='0'
                ang='3'
            else:
                lin='1'
                ang='0'
            if ang[-1]!='0':
                ang_coeff='0010'
                send(ang_coeff+ang,sock)
            if lin[-1]!='0':
                if dist<20:
                    send('0005'+lin,sock)
                else:
                    send('0020'+lin,sock)
         
            cv2.line(frame,centre ,pos_list[go_for],(0,105,255),2)
            #cv2.putText(frame,'distance:'+str(dist), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255),2)
    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) & 0xFF ==ord('q'):    
        break
 
# cleanup the camera and close any open windows
sock.close()
cap.release()
cv2.destroyAllWindows()
        
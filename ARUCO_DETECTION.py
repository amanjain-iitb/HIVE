import numpy as np
import cv2
import cv2.aruco as aruco
import math

# Define Tag
id_to_find=19
marker_size=10

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
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

aruco_dict=aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters=aruco.DetectorParameters_create()

#cap=cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

font=cv2.FONT_HERSHEY_PLAIN




cap = cv2.VideoCapture("http://192.168.0.137:8080/video")

def make_480p():
    cap.set(3,640)
    cap.set(4,480)

make_480p()


while True:
    ret,frame=cap.read()
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    corners,ids,rejected=aruco.detectMarkers(image=gray,dictionary=aruco_dict,parameters=parameters,cameraMatrix=camera_matrix,distCoeff=camera_distortion)
    print(corners,ids,rejected)
    
    if corners!= []:
        if ids[0]==id_to_find:
            ret=aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        
            rvec,tvec=ret[0][0,0,:],ret[1][0,0,:]
        
            aruco.drawDetectedMarkers(frame,corners)
            aruco.drawAxis(frame,camera_matrix,camera_distortion,rvec,tvec,10)
        
            str_position = "MARKER Position x=%4.0f y=%4.0f z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0,100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc = R_ct.T
            
            #Get the attitude in terms of Euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
            
            #Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f, p=%4.0f, y=%4.0f" % (math.degrees(roll_marker), math.degrees(pitch_marker), math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0,250), font, 1, (0,255,0), 2, cv2.LINE_AA)
            
            ''' #Now get the position and attitude of the camera with respect to marker
            pos_camera = R_tc*np.matrix(tvec)
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            
            strPosCam = "CAMERA Pose x=%4.0f, y=%4.0f, z=%4.0f" % (pCamera[0], pCamera[1], pCamera[2])
            cv2.putText(frame, strPosCam, (0,200), font, 1, (0,255,0), 2, cv2.LINE_AA)
            
            strAttCam = "Camera Att r=%4.0f, p=%4.0f, y=%4.0f" % (math.degrees(roll_camera), math.degrees(pitch_camera), math.degrees(yaw_camera))
            cv2.putText(frame, strPosCam, (0,200), font, 1, (0,255,0), 2, cv2.LINE_AA)
            '''
    #Display the frame
    cv2.imshow('frame',frame)
    key=cv2.waitKey(1) & 0xFF
    if key==ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
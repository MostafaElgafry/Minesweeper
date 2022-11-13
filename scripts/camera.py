#!/usr/bin/env python
import numpy as np
import cv2
import cv2.aruco as aruco
import  math
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import rospy
from minesweeper.msg import Pose_msg
from minesweeper.srv import *
######################################################

rospy.init_node("camera_node",anonymous=False)
pub=rospy.Publisher("/minesweeper/camera_pose",Pose_msg,queue_size=10)

########################################################

camera = PiCamera()
camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(800, 600))
time.sleep(.1)

marker_size =  24.9 # - [cm]


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


# --- Get the camera calibration path
calib_path = ""
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')


# --- 180 deg rotation matrix around the x axis
R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

# --- Define the aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

# --- Capture the videocamera (this may also be a video or a picture)

# -- Set the camera size as the one it was calibrated with
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# -- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
rvec = []
tvec = []
dim = 500
ids_to_find = {
    1: [0, 0, 0],
    2: [dim/2, 0, math.pi*.5],
    3: [dim, 0, math.pi],
    4: [dim, dim/2, math.pi],
    5: [dim, dim, math.pi],
    6: [dim/2, dim, math.pi*1.5],
    7: [0, dim, 0],
    8: [0, dim/2, 0]
}

for image in camera.capture_continuous(rawCapture, format="bgr",\
        use_video_port = True, resize=(800,600)):
    # -- Read the camera frame
    frame = image.array
    #imgResp = urlopen(url)
    #imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
    #frame = cv2.imdecode(imgNp, -1)

    # -- Convert in gray scale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # -- remember, OpenCV stores color images in Blue, Green, Red

    # -- Find all the aruco markers in the image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
    if ids is not None :
        for i in range(0,len(ids)) :
            if not ids[i] in range(1, 9):
                continue
            ret = aruco.estimatePoseSingleMarkers(corners[i], marker_size, camera_matrix, camera_distortion)
            rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            # -- Obtain the rotation matrix tag->camera
            R_ct = np.matrix(cv2.Rodrigues(rvec)[0])

            R_tc = R_ct.T  # Rows to couloms vice verse

            # -- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc * np.matrix(tvec).T

            # -- Print the marker's attitude respect to camera frame
            #pos_camera[0], pos_camera[1], pos_camera[2] = (pos_camera[2], pos_camera[0], pos_camera[1])

            temp_pos = pos_camera
            pos_camera = [temp_pos[2], temp_pos[0], temp_pos[1]]
            
            ######################
            ########################

            #str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
            #    pos_camera[0], pos_camera[1], pos_camera[2])
            #cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)

            #str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
            #    math.degrees(roll_camera), math.degrees(pitch_camera),
            #    math.degrees(yaw_camera))
            #cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            num= int(ids[i])
            pos_marker = ids_to_find[num]
            radius = np.hypot(pos_camera[0], pos_camera[1])

            theta = math.atan2(pos_camera[1], pos_camera[0])
            theta += ids_to_find[num][2]
            pos_camera[0] = radius*math.cos(theta)
            pos_camera[1] = radius*math.sin(theta)
            # relative to the marker
            pos_camera[0], pos_camera[1] = pos_camera[0]+pos_marker[0], pos_camera[1]+pos_marker[1]
            #Robot_position = "Robot Position x=%4.0f  y=%4.0f " % (
            #    pos_camera[0], pos_camera[1] )
            #cv2.putText(frame, Robot_position , (0, 200), font, 1, (255, 0, 0), 2, cv2.LINE_AA)

            #####
            x=pos_camera[0]
            y=pos_camera[1]
            msg=Pose_msg()
            msg.x=x
            msg.y=y
            msg.theta = theta
            global pub
            pub.publish(msg)
            '''
            if ids[i] == 72 :
                rvec72, tvec72 = ret[0][0, 0, :], ret[1][0, 0, :]
                # -- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec72)[0])

                R_tc = R_ct.T  # Rows to couloms vice verse

                # -- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc * np.matrix(tvec72).T

                # -- Print the marker's attitude respect to camera frame
                str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                pos_camera[0], pos_camera[1], pos_camera[2])
                cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)

                str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                    math.degrees(roll_camera), math.degrees(pitch_camera),
                    math.degrees(yaw_camera))
                cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec72, tvec72, 10)

            if ids[i] == 62 :
                rvec62, tvec62 = ret[0][0, 0, :], ret[1][0, 0, :]
                # -- Obtain the rotation matrix tag->camera
                R_ct = np.matrix(cv2.Rodrigues(rvec62)[0])

                R_tc = R_ct.T  # Rows to couloms vice verse

                # -- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc * np.matrix(tvec62).T

                # -- Print the marker's attitude respect to camera frame
                str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f" % (
                    pos_camera[0], pos_camera[1], pos_camera[2])
                cv2.putText(frame, str_position, (0, 200), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                # -- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip * R_tc)

                str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f" % (
                    math.degrees(roll_camera), math.degrees(pitch_camera),
                    math.degrees(yaw_camera))
                cv2.putText(frame, str_attitude, (0, 250), font, 1, (255, 0, 0), 2, cv2.LINE_AA)

                aruco.drawAxis(frame, camera_matrix, camera_distortion, rvec62, tvec62, 10)
                '''
            break ###
        # -- Draw the detected marker and put a reference frame over it
        aruco.drawDetectedMarkers(frame, corners)

    # --- Display the frame
    #cv2.imshow('frame', frame)

    # --- use 'q' to quit
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    rawCapture.truncate(0)

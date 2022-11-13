import numpy as np
import cv2
import cv2.aruco as aruco
import  math
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

camera = PiCamera()
camera.framerate = 32

rawCapture = PiRGBArray(camera, size=(800, 600))
time.sleep(.1)


#url = 'http://192.168.11.17:8080/shot.jpg'  # Not fixed ,every time change
#cap = cv2.VideoCapture(0)
#cap.set(3,1280)
#cap.set(4,720)

nSnap = 0

#w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
#h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
w = 800
h = 600

name = 'snap shot'
fileName = "%s_%d_%d_" % (name, w, h)
for image in camera.capture_continuous(rawCapture, format="bgr",\
        use_video_port = True, resize=(800,600)):
    # -- Read the camera frame
    frame = image.array
    # ret, frame = cap.read()
    #imgResp = urlopen(url)
    #imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
    #frame = cv2.imdecode(imgNp, -1)
    cv2.imshow('camera', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    if key == ord(' '):
        print("Saving image ", nSnap)
        cv2.imwrite("%s%d.jpg" % (fileName, nSnap), frame)
        nSnap += 1
    rawCapture.truncate(0)

#cap.release()
cv2.destroyAllWindows()

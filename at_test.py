import cv2
import numpy as np
import Apriltag
from Apriltag import Detector


cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_BUFFERSIZE,1)
at_detector = Detector(searchpath=['apriltags/lib', 'apriltags/lib64'],
                           families='tag36h11',
                           nthreads=1,
                           quad_decimate=2.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
if not cam.isOpened():
    print("cannot access camera 0")
    exit()

print("start detecting")
while True:
    # read from camera
    ret, frame = cam.read()
    if not ret:
        break

    # detect on greyscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (640, 360))
    # cv2.imshow('frame', gray)

    detections = at_detector.detect(gray)
    # format output
    if len(detections) >= 1:
        for idx in range(len(detections)):
            print("Detected tag id[" + str(detections[idx].tag_id), end='] @ ')
            print('x = '  + str(round(detections[idx].center[0])) +
                  ' y = ' + str(round(detections[idx].center[1])) )
            print(dir(detections[idx]), '\n\n')

    # break
    if cv2.waitKey(1) == ord('q'):
        break


# When everything done, release the capture
cam.release()
cv2.destroyAllWindows()


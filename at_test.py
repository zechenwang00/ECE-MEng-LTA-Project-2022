import cv2
import numpy as np
import Apriltag
from Apriltag import Detector
import math


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

# width = cam.get(3)
# height = cam.get(4)
# print(width, height)

print("start detecting")
while True:
    # read from camera
    ret, frame = cam.read()
    if not ret:
        break

    # camera info, resolution 640*480
    focal = 528.7
    cx = 320
    cy = 240


    # detect on greyscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('frame', gray)

    detections = at_detector.detect(gray, estimate_tag_pose=True,
                                          camera_params=[focal, focal, cx, cy],
                                          tag_size=0.175)
    # format output
    if len(detections) >= 1:
        for idx in range(len(detections)):
            # process tag info
            tag_id = detections[idx].tag_id
            center_x_pixel = detections[idx].center[0]
            center_y_pixel = detections[idx].center[1]

            # process translation/rotation
            translation_vector = detections[idx].pose_t
            # rotation_vector  = detections[idx].pose_R
            # err_vector       = detections[idx].pose_err
            x_translation = translation_vector[0][0]
            y_translation = translation_vector[1][0]
            z_translation = translation_vector[2][0]

            # print output
            print("Detected tag id[", tag_id, ']')
            # print('x center pixel: ', center_x_pixel,' y center pixel: ', center_y_pixel)
            print("x:{:.2f} ".format(x_translation),
                  "y:{:.2f} ".format(y_translation),
                  "z:{:.2f}" .format(z_translation))

            # print("pose_R:", detections[idx].pose_R)
            # print("pose_T:", detections[idx].pose_t)
            # print("pose_err:", detections[idx].pose_err)

            print("\n")

    # break
    if cv2.waitKey(1) == ord('q'):
        break


# When everything done, release the capture
cam.release()
cv2.destroyAllWindows()
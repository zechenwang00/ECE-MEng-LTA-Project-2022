import socket
from multiprocessing import Process,Pipe
import time
import control as co
import flightsensor as sensor
import Apriltag 
from Apriltag import Detector
import cv2
import math

serverPort = 12002

serverSocket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
serverSocket.bind(('10.49.1.249',serverPort))
serverSocket.setblocking(False)
serverSocket.listen(1)
return_msg = ''

running = True          # Looping variable
wp = [0,0,1,0]          # Current waypoint
op_mode = 0             # 0: full auto, 1: waypoint, 2: manual
AT_visible = True      # True: AprilTag currently in view of camera, False: otherwise
AT_ID = 1               # ID of last visible AprilTag
AT_dist = 1           # Last measured distance to AprilTag in meters
AT_ang = 0             # Last measured angle of AprilTag
AT_coords = {1:[0,0],\
             2:[1,3],\
             4:[2,-1],\
             5:[-2,1],\
             3:[1,0],\
             4:[2,0],\
             7:[0,1],\
             6:[-1,-1]}   # Dictionary mapping AprilTag IDs to known (x,y) coordinates
AT_seq = 0
             
# Time of flight distances
# Left, center, right
TOF_dist = [5.8,1.5,0.1]

# Time of flight status
# Left, center, right
# 0: OK
# 1: Danger
# 2: Collision imminent
TOF_status = [0, 0, 0]
#Apriltag section
# visualization = True
at_detector = Detector(searchpath=['apriltags/lib', 'apriltags/lib64'],
                           families='tag36h11',
                           nthreads=1,
                           quad_decimate=2.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

# Apriltag vars
x_translation = 0
y_translation = 0
z_translation = 0
x_theta = 0
tag_id = 0

# camera setup
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_BUFFERSIZE,1)

# camera info, resolution 640*480
# focal length in pixels
# focal = 528.7 # for pi cam
focal = 800  # for usb cam
# center of camera in pixels
cx = int(cam.get(3)) / 2
cy = int(cam.get(4)) / 2

if not cam.isOpened():
    print("cannot access camera 0")
    exit()
print("The server is ready to receive")
num=0
try:
    while running:
        try:
            connectionSocket, addr = serverSocket.accept()
            message = connectionSocket.recv(1024).decode()
            print("received message: " + message)
        except:
            message = None

        # read camera
        ret, img = cam.read()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if num==5:
            # reset
            num=0
            print('detecting tags')
            detections = at_detector.detect(img, estimate_tag_pose=True,
                                            camera_params=[focal, focal, cx, cy],
                                            tag_size=0.175)
            if detections:
                # increment AT sequence number
                AT_seq = AT_seq + 1
                for idx in range(len(detections)):
                    # process tag info
                    tag_id = detections[idx].tag_id
                    center_x_pixel = detections[idx].center[0]
                    center_y_pixel = detections[idx].center[1]

                    # process translation/rotation
                    translation_vector = detections[idx].pose_t
                    rotation_matrix  = detections[idx].pose_R
                    # err_vector       = detections[idx].pose_err
                    x_translation =  translation_vector[0][0]
                    y_translation = -translation_vector[1][0]  # y is inverted for usb cam
                    z_translation =  translation_vector[2][0]

                    # swap the mathematical x_theta and y_theta, ignore z for now
                    x_theta = math.atan2(-rotation_matrix[2, 0],
                                         math.sqrt(rotation_matrix[2][1] ** 2 + rotation_matrix[2][2] ** 2))
                    y_theta = math.atan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

                    # print output
                    print("Detected tag id[", tag_id, ']')
                    # print('x center pixel: ', center_x_pixel,' y center pixel: ', center_y_pixel)
                    print("x:{:.2f} ".format(x_translation),
                          "y:{:.2f} ".format(y_translation),
                          "z:{:.2f}".format(z_translation),
                          "x_theta:{:.2f}".format(x_theta),
                          "y_theta:{:.2f}".format(y_theta))

                    # print("pose_R:", detections[idx].pose_R)
                    # print("pose_T:", detections[idx].pose_t)
                    # print("pose_err:", detections[idx].pose_err)

                    print("\n")
        num=num+1
        if AT_ang >180 or AT_ang <-180:
            AT_ang=0

        if message is None:
            continue
        if message == "quit":
            # Close server (debugging)
            return_msg = "quitting"
            co.quitserver()
            cam.release()
            cv2.destroyAllWindows()
            running = False
        elif message == "forward":
            co.forward()
            # Manual command move forward
            pass
        elif message == "backward":
            co.backward()
            # Manual command move backward
            pass
        elif message == "right":
            #AT_ang=AT_ang+5
            co.right()
            # Manual command pivot right
            pass
        elif message == "left":
            #AT_ang=AT_ang-5
            co.left()
            # Manual command pivot left
            pass
        elif message == "up":
            #AT_dist=AT_dist+0.1
            co.top()
            # Manual command increase altitude
            pass
        elif message == "down":
            #AT_dist=AT_dist-0.1
            co.bot()
            # Manual command decrease altitude
            pass
        elif message == "stop":
            co.stopmotor()
            # Manual command stop motors
            pass
        elif message == "curr wp":
            # Requesting current waypoint
            return_msg = "{:.2f}".format(x_translation) + "," + "{:.2f}".format(y_translation) + "," + "{:.2f}".format(z_translation) + "," + str(tag_id)
            print(return_msg)
        elif message.split(' ')[0] == "mode":
            op_mode = int(message.split(' ')[1])
            print("new op mode: " + str(op_mode))
        elif message.split(' ')[0] == "wp":
            # Waypoint command set new waypoint
            wp[:] = [int(i) for i in message.split(' ')[1].split(',')]

        elif message == "at":
            # Requesting AprilTag data
            return_msg = str(AT_visible)
            if AT_visible and (AT_ID in AT_coords):
                return_msg += "," + str(tag_id) + "," + "{:.2f}".format(x_translation) + "," \
                                                      + "{:.2f}".format(y_translation) + "," \
                                                      + "{:.2f}".format(z_translation) + "," \
                                                      + "{:.2f}".format(x_theta) + "," \
                                                      + "{:d}".format(AT_seq)
        elif message == "tof":
            TOF_dist=sensor.measurement()
            # update TOF status
            warning_dist_red = 700
            warning_dist_yellow = 1000
            for idx, reading in enumerate(TOF_dist):
                if reading < 700:
                    TOF_status[idx] = 2     # red
                elif reading < 1000:
                    TOF_status[idx] = 1     # yellow
                else:
                    TOF_status[idx] = 0     # green
            # Requesting time of flight data
            return_msg = ""
            for n in range(3):
                return_msg = return_msg + str(TOF_dist[n]) + ","
            for n in range(3):
                return_msg = return_msg + str(TOF_status[n]) + ","
            return_msg = return_msg[:-1]
        else:
            print("Not recognized")
            return_msg = "Not recognized"
        connectionSocket.send(return_msg.encode())
        connectionSocket.close()
    print ("running = {}".format(running))
    serverSocket.close()
except:
    serverSocket.close()
    raise

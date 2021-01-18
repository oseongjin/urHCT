"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""

import URBasic
from threading import Thread
import math
import numpy as np
import sys
import cv2
import time
import imutils
import socket
import pickle
from imutils.video import VideoStream
import math3d as m3d

"""SETTINGS AND VARIABLES ________________________________________________________________"""
import picamera
from picamera.array import PiRGBArray
global token
token = None   #setting Global Token Value 


#pretrained_model = cv2.dnn.readNetFromCaffe("MODELS/deploy.prototxt.txt", "MODELS/res10_300x300_ssd_iter_140000.caffemodel")
pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml', 'models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024, 768)  # resolution the video capture will be resized to, smaller sizes can speed up detection
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio  = video_resolution[0] / video_resolution[1]  # Aspect ration of each frame
video_viewangle_hor = math.radians(25)  # Camera FOV (field of fiew) angle in radians in horizontal direction
#video_viewangle_vert = video_viewangle_hor / video_asp_ratio  #  Camera FOV (field of fiew) angle in radians in vertical direction
max_x = 0.2
max_y = 0.2
'''
vs = VideoStream(src= 0 ,
                 usePiCamera= RASPBERRY_BOOL,
                 resolution=video_resolution,
                 framerate = 13,
                 meter_mode = "backlit",
                 exposure_mode ="auto",
                 shutter_speed = 8900,
                 exposure_compensation = 2,
                 rotation = 0).start()
'''
vs = VideoStream(src= 0 ,
                 usePiCamera= True,
                 resolution=video_resolution).start()
time.sleep(0.2)



"""FUNCTIONS _____________________________________________________________________________"""
def send_Data():
    global token
    HOST = '10.0.0.100'
    PORT = 7777
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client_socket.connect((HOST,PORT))
    while True:
        time.sleep(0.2)
        data = pickle.dumps(token)
        client_socket.sendall(data)
        data = None
        token = None
def find_faces_dnn(image):
    """
    Finds human faces in the frame captured by the camera and returns the positions
    uses the pretrained model located at pretrained_model

    Input:
        image: frame captured by the camera

    Return Values:
        face_centers: list of center positions of all detected faces
            list of lists with 2 values (x and y)
        frame: new frame resized with boxes and probabilities drawn around all faces

    """
    frame = image
    frame = imutils.resize(frame, width= video_resolution[0])
	
    global token
    temp = 0
    save =[]

    #blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
    #                             (300, 300), (104.0, 177.0, 123.0))
    blob = cv2.dnn.blobFromImage(frame, size=(300, 300), ddepth=cv2.CV_8U)
    # pass the blob through the network and obtain the detections and predictions
    pretrained_model.setInput(blob)

    # the following line handles the actual face detection
    # it is the most computationally intensive part of the entire program
    # TODO: find a quicker face detection model
    detections = pretrained_model.forward()
    face_centers = []
    # loop over the detections
    for s in detections.reshape(-1,7):
        confidence = float(s[2])
        fstartX = int(s[3] * frame.shape[1])
        fstartY = int(s[4] * frame.shape[0])
        fendX = int(s[5] * frame.shape[1])
        fendY = int(s[6] * frame.shape[0])
		
        if confidence < 0.8:
            continue 

        else:
            # compute the (x, y)-coordinates of the bounding box for the object
            pred_boxpts = (fstartX,fstartY,fendX,fendY)
            save.append(pred_boxpts)
    print(save)

    storge = []
    endSave = []
    if save != []:
        for i in save:
            storge.append(i[2]-i[0])
            endSave.append(i[2])
        first_max = max(storge)
        first_index = storge.index(first_max)
        storge[first_index] = 0
        second_max = max(storge)
        second_index = storge.index(second_max)
        if first_max - second_max < 100:
            if endSave[first_index] > endSave[second_index]:
                winner = save[first_index]
                startX = winner[0]
                startY = winner[1]
                endX = winner[2]
                endY = winner[3]
            else:
                winner = save[second_index]
                startX = winner[0]
                startY = winner[1]
                endX = winner[2]
                endY = winner[3]
        else:
            winner = save[first_index]
            startX = winner[0]
            startY = winner[1]
            endX = winner[2]
            endY = winner[3]
        print('first')		    
        data = (startX,startY,endX,endY)
        # draw the bounding box of the face along with the associated probability
        text = "{:.2f}%".format(confidence * 100)
        y = startY - 10 if startY - 10 > 10 else startY + 10
        face_center = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
        sub = (endX - startX)/2
        dis =300 - (endX - startX)
        position_from_center = (face_center[0] - video_midpoint[0], face_center[1] - video_midpoint[1],dis)
        print('second')
        if endX - startX < 90:
            token = None
            return 0 
        print('third')
        #if -sub<position_from_center[0]<sub and -sub < position_from_center[1] <sub :
        #    face_centers.append((0,0))
        #else:
        face_centers.append(position_from_center)
        token = face_centers
	        
        return face_centers,face_center,text,data,y 
    else:
        return 0 
        print('else')

def show_frame(frame):
    cv2.imshow('img', frame)
    k = cv2.waitKey(6) & 0xff


def set_lookorigin():
    """
    Creates a new coordinate system at the current robot tcp position.
    This coordinate system is the basis of the face following.
    It describes the midpoint of the plane in which the robot follows faces.

    Return Value:
        orig: math3D Transform Object
            characterises location and rotation of the new coordinate system in reference to the base coordinate system

    """
    position = robot.get_actual_tcp_pose()
    orig = m3d.Transform(position)
    return orig


"""FACE TRACKING LOOP ____________________________________________________________________"""
time.sleep(1)
th = Thread(target = send_Data)
th.start()
try:
    print("starting loop")
    while True:
        frame = vs.read()
        image_for_result = frame.copy()
        if find_faces_dnn(frame) == 0:
            pass
        else:
            face_positions,face_center,text,data,y = find_faces_dnn(frame)
            startX = data[0]
            startY = data[1]
            endX = data[2]
            endY = data[3]
            cen = (startX + endX )/2
            cv2.rectangle(image_for_result, (startX, startY), (endX, endY),
                          (0, 0, 255), 2)
            cv2.putText(image_for_result, text, (startX, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
            #cv2.putText(frame, str(position_from_center), face_center, 0, 1, (0, 200, 0))
            cv2.line(image_for_result, video_midpoint, face_center, (0, 200, 0), 5)
            cv2.circle(image_for_result, face_center, 4, (0, 200, 0), 3)
            cv2.circle(image_for_result,(int(cen),int(startY)+ int((endY - startY)/10)),3,(255,0,0),3)
        show_frame(image_for_result)
    print("exiting loop")
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect


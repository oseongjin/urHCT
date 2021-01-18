"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""
#______MEMEO__________ : set detection distance change short__________________________#

from collections import deque
#from mpg123 import Mpg123 , Out123
import URBasic
import math
import pickle
import numpy as np
import sys
import cv2
import time
import serial
import math3d as m3d
import socket
import imutils
from imutils.video import VideoStream
from threading import Timer ,Thread

"""SETTINGS AND VARIABLES ________________________________________________________________"""


pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml','models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024,768)
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio = video_resolution[0]/video_resolution[1]
video_viewangle_hor = math.radians(25) 

face_distance = 400
before_positions = None
vs = VideoStream(src = 0,
                 usePiCamera = True,
                 resolution=video_resolution).start()
time.sleep(2)

"""FUNCTIONS _____________________________________________________________________________"""
"""
      Finish Robot move
   """

def find_faces_dnn(image):
    global before_positions
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

    save =[]

   
    blob = cv2.dnn.blobFromImage(frame, size=(300, 300), ddepth=cv2.CV_8U)
    pretrained_model.setInput(blob)
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
        data = (startX,startY,endX,endY)
        # draw the bounding box of the face along with the associated probability
        text = "{:.2f}%".format(confidence * 100)
        y = startY - 10 if startY - 10 > 10 else startY + 10
        face_center = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
        sub = (endX - startX)/2
        dis =face_distance - (endX - startX)
        position_from_center = (face_center[0] - video_midpoint[0], face_center[1] - video_midpoint[1],dis)
        if endX - startX < 160:
            return before_positions,frame
        face_centers.append(position_from_center)
        cen = (startX + endX )/2
        cv2.rectangle(frame, (startX, startY), (endX, endY),
                      (0, 0, 255), 2)
        cv2.putText(frame, text, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        cv2.line(frame, video_midpoint, face_center, (0, 200, 0), 5)
        cv2.circle(frame, face_center, 4, (0, 200, 0), 3)
        cv2.circle(frame,(int(cen),int(startY)+ int((endY - startY)/10)),3,(255,0,0),3)
        before_positions = face_centers

        return face_centers,frame
    
    else:
        #print('else')
        return before_positions,frame

def sendPoint(point):
    #print(point)
    data = pickle.dumps(point)
    point_sock.sendall(data)

"""FACE TRACKING LOOP ____________________________________________________________________"""

print("starting loop")
while True:
    frame = vs.read()
    face_positions , new_frame = find_faces_dnn(frame)
    #show_frame(new_frame)
    #print('face_positions',face_positions)
    if len(face_positions) == 0:
        print("pass")
        pass
    else:
        sendPoint(face_positions)

print("exiting loop")

   








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

POINT_IP = "192.168.0.200"
POINT_PORT = 4001

DSP_IP = "192.168.0.38"
DSP_PORT = 5001

pretrained_model = cv2.dnn.readNet('models/landmarks-regression-retail-0009.xml','models/landmarks-regression-retail-0009.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024,768)
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio = video_resolution[0]/video_resolution[1]
video_viewangle_hor = math.radians(25) 

face_distance = 400
before_positions = ""
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
    for s in detections.reshape(-1,10):
        
        #confidence = float(s[2])
        lx = int(s[0] * frame.shape[1])
        ly = int(s[1] * frame.shape[0])
        rx= int(s[2] * frame.shape[1])
        ry = int(s[3] * frame.shape[0])

    leftEye = (ly, ly)
    rightEye = (rx, ry)
    cv2.circle(frame, leftEye, 4, (0, 200, 0), 3)
    cv2.circle(frame, rightEye, 4, (0, 200, 0), 3)

    return face_centers,frame
    

def sendPoint(point):
    #print(point)
    data = pickle.dumps(point)
    point_sock.sendall(data)

def show_frame(frame):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]
    result, imgencode = cv2.imencode(".jpg",frame, encode_param)
    #압축된 이미지를 numpy 형태의 array로 변환후 string 으로 변환
    data = np.array(imgencode)
    stringData = data.tostring()
    #전송의 안정성을 위해 먼저 전송할 데이터의 크기를 전송
    tmpstr = str(len(stringData)).ljust(16)
    try:
        dsp_sock.send( tmpstr.encode("utf-8"))
        #수신준비된 서버에 스트링 포맷의 프레임 데이터 전송
        dsp_sock.send(stringData)
    except:
        print("Connection has been disconnected")
        dsp_sock.close()

"""FACE TRACKING LOOP ____________________________________________________________________"""

print("starting loop")

point_sock = socket.socket()
point_sock.connect((POINT_IP,POINT_PORT))

dsp_sock = socket.socket()
dsp_sock.connect((DSP_IP,DSP_PORT))
while True:
    frame = vs.read()
    face_positions , new_frame = find_faces_dnn(frame)
    show_frame(new_frame)
    #print('face_positions',face_positions)
    if len(face_positions) == 0:
        print("pass")
        pass
    else:
        sendPoint(face_positions)

print("exiting loop")

   








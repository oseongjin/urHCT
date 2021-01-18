#!/usr/bin/env python

import os, sys, time
import socket, serial, logging
import imutils, codecs
import json
import math
import math3d as m3d
import pickle
import cv2
import numpy as np

import URBasic

from collections import deque
from imutils.video import VideoStream
from threading import Timer ,Thread

#------------ System logging Setting ----------------------------
rblogger = logging.getLogger("robot")
rblogger.setLevel(logging.INFO)

stream_hander = logging.StreamHandler()
rblogger.addHandler(stream_hander)

#file_handler = logging.FileHandler('robot.log')
#rblogger.addHandler(file_handler)

#------------SETTINGS AND VARIABLES -------------------------
if os.path.isfile('robotcfg.dat'):
    with open('robotcfg.dat', 'r') as jfp:
        cfgdat=json.load(jfp)
else:
    rblogger.error("robotcfg.dat not exist!! program stop!!")
    exit()

RBCTRL_IP = cfgdat['RBCTRL_IP']
RBCTRL_PORT = cfgdat['RBCTRL_PORT']

DSP_IP = cfgdat['DSP_IP']
DSP_PORT = cfgdat['DSP_PORT']

rblogger.debug("RB CTRL IP : {}".format(RBCTRL_IP))
rblogger.debug("DISPLAY IP : {}".format(DSP_IP))

camCmd = [ 0xff, 0xff, 0xdc, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 ] # only read data

camhost = 'localhost' 
camport = 5011

#------------- Function and routine -----------------------

pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml','models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)


landmark_pretrained_model = cv2.dnn.readNet('models/facial-landmarks-35-adas-0002.xml','models/facial-landmarks-35-adas-0002.bin')
landmark_pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (640, 480)
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio = video_resolution[0]/video_resolution[1]
video_viewangle_hor = math.radians(25) 

face_distance = 400   #400
before_positions = ""
vs = VideoStream(src = 0,
                 usePiCamera = True,
                 resolution=video_resolution).start()
time.sleep(2)

"""FUNCTIONS _____________________________________________________________________________"""
"""
      Finish Robot move
"""
   
def find_faces_dnn(image, tddata):
    
    global before_positions
    save =[]
    face_centers = []
    storge = []
    endSave = []
    frame = image
    frame = imutils.resize(frame,width = video_resolution[0]) 
        
    blob = cv2.dnn.blobFromImage(frame, size=(300, 300), ddepth=cv2.CV_8U)
    pretrained_model.setInput(blob)
    detections = pretrained_model.forward()

    for s in detections.reshape(-1,7):          # detections = [image_id, label, conf, xmin, ymin, xmax, ymax]
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

        rblogger.debug("Angle Box xy data {}".format(data))
        try:
        
            land_frame = frame[startY:endY,startX:endX]
            land_blob = cv2.dnn.blobFromImage(land_frame, size= (60, 60), ddepth=cv2.CV_8U)
            landmark_pretrained_model.setInput(land_blob)
            land_result = landmark_pretrained_model.forward()

            for q in land_result.reshape(-1,70):
                lx = int(q[0] * land_frame.shape[1])
                ly = int(q[1] * land_frame.shape[0])
                rx = int(q[4] * land_frame.shape[1])
                ry = int(q[5] * land_frame.shape[0])
            #rblogger.debug("land_result {}".format(land_result))
            
            leftEye = (lx, ly)
            rightEye = (rx, ry)
            center_eye = (int((lx + rx)/2), int((ly + ry)/2))
            final_eye = (startX + center_eye[0], startY + center_eye[1])
            rblogger.debug("leftEye : {}, rightEye : {}".format(leftEye, rightEye))
            
            # draw the bounding box of the face along with the associated probability
            distance_text = "distance : " + tddata[2] + "mm"
            #tmp_font = (tddata[3] * 1.1) / 100
            temp_text = "temp : " + tddata[3] 
            y = startY - 10 if startY - 10 > 10 else startY + 10
            face_center = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
            sub = (endX - startX)/2
            dis =face_distance - (endX - startX)
            dist  = tddata[2]
            htemp = tddata[3]

            position_from_center = (final_eye[0] - video_midpoint[0], final_eye[1] - video_midpoint[1], dis, dist, htemp)
            if endX - startX < 160:
                return before_positions,frame
                
            face_centers.append(position_from_center)
            cen = (startX + endX )/2
            cv2.circle(land_frame, center_eye, 4, (255, 0, 0), 3)

            cv2.rectangle(frame, (startX, startY), (endX, endY),
                          (0, 0, 255), 2)
            cv2.putText(frame, distance_text, (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            cv2.putText(frame, temp_text, (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            cv2.line(frame, video_midpoint, final_eye, (0, 200, 0), 5)
            before_positions = face_centers

        except Exception as e:
            rblogger.debug("Find Face DNN routin exception {}".format(e))
            
        return face_centers, frame
    
    else:
        rblogger.debug('else')
        return before_positions, frame

def sendPoint(rbpt_conn, point):
    rblogger.debug("point {}".format(point))
    data = pickle.dumps(point)
    rbpt_conn.sendall(data)

def show_frame(show_sock, frame):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY),90]
    result, imgencode = cv2.imencode(".jpg",frame, encode_param)

    data = np.array(imgencode)
    stringData = data.tostring()

    tmpstr = str(len(stringData)).ljust(16)
    try:
        show_sock.send( tmpstr.encode("utf-8"))
        show_sock.send(stringData)
    except:
        show_sock.close()
        rblogger.error("Connection has been disconnected")

"""FACE TRACKING LOOP ____________________________________________________________________"""

rblogger.info("starting loop")

point_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
point_sock.connect((RBCTRL_IP,int(RBCTRL_PORT)))

dsp_sock = socket.socket()
dsp_sock.connect((DSP_IP, int(DSP_PORT)))

cnt = 0
tmpRcv = ["0","0","0","0"]

cam_sock = socket.socket()
cam_sock.connect((camhost,camport))
sndStrmsg = bytes(camCmd).hex()

while True:
    try:
        if cnt == 5:
            cam_sock.sendall(bytes(sndStrmsg.encode("utf-8")))
            received_data = codecs.decode(cam_sock.recv(1024))
            tmpRcv = received_data.split(',')
            cnt = 0
        
        frame = vs.read()
        face_positions , new_frame = find_faces_dnn(frame, tmpRcv)
        show_frame(dsp_sock, new_frame)
        rblogger.debug('face_positions {}'.format(face_positions))
                
        if len(face_positions) == 0:
            rblogger.info("pass")
            pass
        else:
            sendPoint(point_sock, face_positions)
            pass
        cnt += 1
        
    except:
        rblogger.info("main loop exception ")
        cam_sock.close()
        dsp_sock.close()
        point_sock.close()
        
rblogger.info("exiting loop")

   

            






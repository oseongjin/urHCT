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

import URBasic

from collections import deque
from imutils.video import VideoStream
from threading import Timer ,Thread

"""SETTINGS AND VARIABLES ________________________________________________________________"""

POINT_IP = "192.168.0.200"
POINT_PORT = 4002

DSP_IP = "192.168.0.43"
DSP_PORT = 5000

sport = serial.Serial("/dev/ttyS0", baudrate=19200, timeout=3.0)

if sport.is_open:
    sport.close()

rdata = bytearray((0xdc, 0x03, 0x00, 0x00, 0x00, 0x04))

pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml','models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)


landmark_pretrained_model = cv2.dnn.readNet('models/facial-landmarks-35-adas-0002.xml','models/facial-landmarks-35-adas-0002.bin')
landmark_pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024,768)
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio = video_resolution[0]/video_resolution[1]
video_viewangle_hor = math.radians(25) 

face_distance = 800   #400
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

        #print("data",data)
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
                nosex = int(q[8] * land_frame.shape[1])
                nosey = int(q[9] * land_frame.shape[0])
            #print("land_result",land_result)
            
            leftEye = (lx, ly)
            rightEye = (rx, ry)
            nose = (nosex, nosey)
            world_nose = (startX + nosex, startY + nosey)
            #print("left : ",leftEye,"rightEye : ",rightEye)
            
            # draw the bounding box of the face along with the associated probability
            distance_text = "distance : " + tddata[2] + "cm"
            temp_text = "temp : " + tddata[3] 
            y = startY - 10 if startY - 10 > 10 else startY + 10
            face_center = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
            sub = (endX - startX)/2
            dis =face_distance - (endX - startX)
            dist  = tddata[2]
            htemp = tddata[3]

            position_from_center = (world_nose[0] - video_midpoint[0], world_nose[1] - video_midpoint[1],dis, dist, htemp)
            if endX - startX < 160:
                return before_positions,frame
            face_centers.append(position_from_center)
            cen = (startX + endX )/2
            cv2.circle(land_frame, leftEye, 4, (255, 0, 0), 3)
            cv2.circle(land_frame, rightEye, 4, (255, 0, 0), 3)
            cv2.circle(land_frame, nose, 4, (255, 0, 0), 3)

            cv2.rectangle(frame, (startX, startY), (endX, endY),
                          (0, 0, 255), 2)
            cv2.putText(frame, distance_text, (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            cv2.putText(frame, temp_text, (50, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
            #cv2.line(frame, video_midpoint, face_center, (0, 200, 0), 5)
            cv2.line(frame, video_midpoint, world_nose, (0, 200, 0), 5)
            #cv2.circle(frame, nose, 4, (0, 200, 0), 3)
            #cv2.circle(frame, face_center, 4, (0, 200, 0), 3)
            #cv2.circle(frame,(int(cen),int(startY)+ int((endY - startY)/10)),3,(255,0,0),3)
            before_positions = face_centers

        except Exception as e:
            print(str(e))
        return face_centers,frame
    
    else:
        #print('else')
        return before_positions,frame

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

cnt = 0
tmpRcv = [0,0,0,0]

while True:
    if cnt == 10:
        sport.open()
        sport.write(rdata)
        received_data = sport.read()              #read serial port
        time.sleep(0.03)
        data_left = sport.inWaiting()             #check for remaining byte
        received_data += sport.read(data_left)
        received_data = received_data.decode("utf-8")
        tmpRcv = received_data.split(',')
        sport.close()
        cnt = 0
    
    frame = vs.read()
    face_positions , new_frame = find_faces_dnn(frame, tmpRcv)
    show_frame(new_frame)
    #print('face_positions',face_positions)
            
    if len(face_positions) == 0:
        print("pass")
        pass
    else:
        sendPoint(face_positions)

    cnt += 1

print("exiting loop")

   








"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""
#______MEMEO__________ : set detection distance change short__________________________#

import URBasic
import math
import pickle
import numpy as np
import sys
import cv2
import time
import math3d as m3d
import socket
import imutils
import argparse
from collections import deque
from imutils.video import VideoStream
from threading import Timer ,Thread

"""SETTINGS AND VARIABLES ________________________________________________________________"""
DSP_IP = "192.168.0.38"
DSP_PORT = 5001

RASP_IP = "192.168.0.200"
RASP_PORT = 5005
detect_point = 2.3
parser = argparse.ArgumentParser(description = "face point move")
parser.add_argument("--point",type=float)
args = parser.parse_args()
if args.point:
    detect_point = args.point

WAITTIME = 20
face_Coord = deque(maxlen = WAITTIME)

pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml','models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024,768)
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio = video_resolution[0]/video_resolution[1]
video_viewangle_hor = math.radians(25) 

before_positions = ""
vs = VideoStream(src = 0,
                 usePiCamera = True,
                 resolution=video_resolution).start()

forward_distance = 750

stopVar = ""

if len(sys.argv) > 1 and sys.argv[1] == "-l":
    _debug = True
else:
    _debug = False


"""FUNCTIONS _____________________________________________________________________________"""

def find_faces_dnn(image):
    global before_positions
    global face_Coord
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
        #face_center = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
        head_detect = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / detect_point))
        #head_detect = (int(startX + (endX - startX) / 2), int((endY - startY) / 10))
        sub = (endX - startX)/2
        dis = forward_distance - (endX - startX)
        #position_from_center = (face_center[0] - video_midpoint[0], face_center[1] - video_midpoint[1],dis)
        position_from_center = (head_detect[0] - video_midpoint[0], head_detect[1] - video_midpoint[1],dis)
        face_centers.append(position_from_center)

        if endX - startX < 160:
            return before_positions,frame
        cen = (startX + endX )/2
        cv2.rectangle(frame, (startX, startY), (endX, endY),
                      (0, 0, 255), 2)
        cv2.putText(frame, text, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        cv2.line(frame, video_midpoint, head_detect, (0, 200, 0), 5)
        #cv2.circle(frame, face_center, 4, (0, 200, 0), 3)
        cv2.circle(frame, head_detect, 4, (0, 200, 0), 3)
        #cv2.circle(frame,(int(cen),int(startY)+ int((endY - startY)/15)) , 4, (0, 200, 0), 3)
        #cv2.circle(frame,(int(cen),int(startY)+ int((endY - startY)/15)),3,(255,0,0),3)
        before_positions = face_centers

        return face_centers,frame
    
    else:
        face_Coord.append(before_positions)
        tmp = False

        if len(face_Coord) == WAITTIME:
            for i in range(int(WAITTIME/2),WAITTIME):
                if face_Coord[int(WAITTIME/2)-1] == face_Coord[i]:
                    tmp = True
                else:
                    tmp = False
                    break
        if tmp == True:
            face_centers = ""        
            before_positions = ""
            face_Coord.clear()
        #print('else_dnn')
        return before_positions,frame
    
        

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

def sendPoint(point):
    #print(point)
    data = pickle.dumps(point)
    point_sock.sendall(data)


def socket_listen():
    HOST = ""
    PORT = 5050
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    server_socket.bind((HOST,PORT))
    server_socket.listen(5)
    while True:
        try:
            #클라이언트 함수가 접속하면 새로운 소켓을 반환한다.
            client_socket, addr = server_socket.accept()
        except KeyboardInterrupt:
            server_socket.close()
            print("Keyboard interrupt")
        print("클라이언트 핸들러 스레드로 이동 됩니다.")
        #accept()함수로 입력만 받아주고 이후 알고리즘은 핸들러에게 맡긴다.
        t = Thread(target=handle_client, args=(client_socket, addr))
        t.daemon = True
        t.start()

def handle_client(client_socket, addr):
    global before_positions
    global stopVar
    print("Connected by",addr)
    rcvData = client_socket.recv(1024)
    rcvCmd = rcvData.decode()
    data = rcvData.decode('utf-8')
    if "stop" in data:
        before_positions = ""
    stopVar = data
    print(data)    

   
    print('handler socket close')

    client_socket.close()

"""FACE TRACKING LOOP ____________________________________________________________________"""

# initialise robot with URBasic

time.sleep(2)
dsp_sock = socket.socket()
dsp_sock.connect((DSP_IP,DSP_PORT))

point_sock = socket.socket()
point_sock.connect((RASP_IP,RASP_PORT))

th1 = Thread(target=socket_listen)
th1.start()
while True:
    frame = vs.read()
    face_positions , new_frame = find_faces_dnn(frame)
    show_frame(new_frame)
    if stopVar == "stop":
        time.sleep(2)
        stopVar = ""
    if len(face_positions) != 0:
        sendPoint(face_positions)

print("exiting loop")
   








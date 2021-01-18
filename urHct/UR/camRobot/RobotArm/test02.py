"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""
#______MEMEO__________ : set detection distance change short__________________________#

from collections import deque
from mpg123 import Mpg123 , Out123
import URBasic
import math
import pickle
import numpy as np
import sys
import cv2
import time
import argparse
import serial
import math3d as m3d
import socket
import imutils
from imutils.video import VideoStream
from threading import Timer ,Thread

"""SETTINGS AND VARIABLES ________________________________________________________________"""

ROBOT_IP = '192.168.0.169'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

DSP_IP = "192.168.0.38"
DSP_PORT = 5001

m_per_pixel = 00.000012  # Variable which scales the robot movement from pixels to meters.
f_per_pixel = 00.000055  #00,000025

startment = '/home/pi/work/rbctrl/startment.mp3'
okment = '/home/pi/work/rbctrl/okment.mp3'
failment = '/home/pi/work/rbctrl/failment.mp3'

tmp355 = "/home/pi/work/rbctrl/355.mp3"
tmp358 = "/home/pi/work/rbctrl/358.mp3"
tmp361 = "/home/pi/work/rbctrl/361.mp3"
tmp364 = "/home/pi/work/rbctrl/364.mp3"
tmp367 = "/home/pi/work/rbctrl/367.mp3"
tmp370 = "/home/pi/work/rbctrl/370.mp3"
tmp373 = "/home/pi/work/rbctrl/373.mp3"
start_effect = "/home/pi/work/rbctrl/start_sound.mp3"
qr_start = "/home/pi/work/rbctrl/qr_pls.mp3"
qr_ok = "/home/pi/work/rbctrl/qr_ok.mp3"

#---waitkey----
UpKey = 2490368
DownKey = 2621440
LeftKey = 2424832
RightKey = 2555904
Space = 32

max_x = 0.15   #Area X
max_y = 0.08    #Area Y
max_z = 0.2    #Area Z
hor_rot_max = math.radians(35)   #Area Camera X     #35
vert_rot_max = math.radians(20)  #Area Camera Y     #20
vector_rot_max = math.radians(15)  #Area Camear Z

eye_cascade = cv2.CascadeClassifier("models/eye_detect.xml")

pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml','models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024,768)
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio = video_resolution[0]/video_resolution[1]
video_viewangle_hor = math.radians(25) 

detect_point = 2.3
stdata = bytearray((0xdc,0x04,0x02,0x02,0x00,0x04))
rddata = bytearray((0xdc,0x04,0x02,0x01,0x01,0x04)) # sensor move front, wing open, sys smail
sdata = bytearray((0xdc,0x03,0x00,0x00,0x00,0x04)) #sysid, read, dumy ,dumy ,dumy ,read cnt,not include CRC,
sport = serial.Serial("/dev/ttyS0", baudrate = 19200, timeout =3.0)

if sport.is_open:
    sport.close()



parser = argparse.ArgumentParser(description = "face point move")
parser.add_argument("--point",type =float)
args = parser.parse_args()
if args.point:
    detect_point = args.point


print("포인트값",args.point)
print("포인트값",detect_point)

before_positions = None
vs = VideoStream(src = 0,
                 usePiCamera = True,
                 resolution=video_resolution).start()

start_check = True
first_time = True
once = True
forward_distance = 750
dc = 0
WAITTIME = 20
xCoord = deque(maxlen = WAITTIME)
yCoord = deque(maxlen = WAITTIME)
time.sleep(2)

if len(sys.argv) > 1 and sys.argv[1] == "-l":
    _debug = True
else:
    _debug = False

if _debug:
    print('test')

"""FUNCTIONS _____________________________________________________________________________"""
"""
      Finish Robot move
   """

def guide(ment):

    mp3 = Mpg123(ment)
    out = Out123()
    
    for frame in mp3.iter_frames(out.start):
        out.play(frame)
    

def distance_check():
    
    global dc
    sport.open()
    
    recvList = []
    sport.write(sdata)
    received_data = sport.read()
    time.sleep(0.03)
    data_left = sport.inWaiting()
    received_data += sport.read(data_left)
    received_data = received_data.decode('ascii')
    save = received_data.split(",")
    print('save',save)
    for i in save:
        recvList.append(i)
    if len(save) != 0 :
        temp = (float(save[3])/0.97 ) * 0.01
        distance = int(save[2])
        print("temp : {} distance : {}".format(temp, distance))
    else : 
        print("sensor value is null")
    sport.close()

    return temp,distance
def writeSerial(order):
    sport.open()
    sport.write(order)
    sport.close()

def check_max_xy(xy_coord):
    """
    Checks if the face is outside of the predefined maximum values on the lookaraound plane

    Inputs:
        xy_coord: list of 2 values: x and y value of the face in the lookaround plane.
            These values will be evaluated against max_x and max_y

    Return Value:
        x_y: new x and y values
            if the values were within the maximum values (max_x and max_y) these are the same as the input.
            if one or both of the input values were over the maximum, the maximum will be returned instead
    """
    x_y = [0,0,0]
    #print("xy before conversion: ", xy_coord)


    if -max_x <= xy_coord[0] <= max_x:
        # checks if the resulting position would be outside of max_x
        x_y[0] = xy_coord[0]
    elif -max_x > xy_coord[0]:
        x_y[0] = -max_x
    elif max_x < xy_coord[0]:
        x_y[0] = max_x
    else:
        raise Exception(" x is wrong somehow:", xy_coord[0], -max_x, max_x)

    if -max_y <= xy_coord[1] <= max_y:
        # checks if the resulting position would be outside of max_y
        x_y[1] = xy_coord[1]
    elif -max_y > xy_coord[1]:
        x_y[1] = -max_y
    elif max_y < xy_coord[1]:
        x_y[1] = max_y
    else:
        raise Exception(" y is wrong somehow", xy_coord[1], max_y)
    #print("xy after conversion: ", x_y)
    if  0 <= xy_coord[2] <= max_z:
        # checks if the resulting position would be outside of max_y
        x_y[2] = xy_coord[2]
    elif  0 > xy_coord[2]:
        x_y[2] =  0  
    elif max_z < xy_coord[2]:
        x_y[2] = max_z
    else:
        raise Exception(" y is wrong somehow", xy_coord[1], max_y)
    #print("xy after conversion: ", x_y)
    return x_y

def find_faces_dnn(image):
    global before_positions
    print("enter dnn")
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
        print("좌표",data)
    
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #roi_gray = gray[startY:startY + endY,startX:startX + endX]
        #roi_color = frame[startY:startY+endY,startX:startX+endX]
        roi_gray = gray[startX:endX,startY:endY]
        roi_color = frame[startX:endX,startY:endY]
        print("gray",len(gray))
        print("roi_gray",len(roi_gray))
        eyes = eye_cascade.detectMultiScale(roi_gray)
        for (ex,ey,ew,eh) in eyes:
            cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        
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
        if endX - startX < 160:
            return before_positions,frame
        face_centers.append(position_from_center)
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



def move_to_face(list_of_facepos,robot_pos):
    """
    Function that moves the robot to the position of the face

    Inputs:
        list_of_facepos: a list of face positions captured by the camera, only the first face will be used
        robot_pos: position of the robot in 2D - coordinates

    Return Value:
        prev_robot_pos: 2D robot position the robot will move to. The basis for the next call to this funtion as robot_pos
    """

    if _debug:
        print('first_line')

    face_from_center = list(list_of_facepos[0])  # TODO: find way of making the selected face persistent

    prev_robot_pos = robot_pos
    #print('before',face_from_center)
    scaled_face_pos = [face_from_center[0] * m_per_pixel,face_from_center[1] * m_per_pixel,face_from_center[2] * f_per_pixel]
    #print('after',scaled_face_pos)

    if _debug:
        print('second_line')
    robot_target_xy = [a + b for a, b in zip(prev_robot_pos, scaled_face_pos)]

    robot_target_xy = check_max_xy(robot_target_xy)
    prev_robot_pos = robot_target_xy

    if _debug:
        print('third_line')
    x = robot_target_xy[0]
    y = robot_target_xy[1]
    z = robot_target_xy[2]    
        
            
    xyz_coords = m3d.Vector(x, y, z)

    x_pos_perc = x / max_x
    y_pos_perc = y / max_y
     
    x_rot = x_pos_perc * hor_rot_max
    y_rot = y_pos_perc * vert_rot_max * -1
     

    tcp_rotation_rpy = [y_rot, x_rot, 0]
    tcp_orient = m3d.Orientation.new_euler(tcp_rotation_rpy, encoding='xyz')
     
    position_vec_coords = m3d.Transform(tcp_orient, xyz_coords)

    oriented_xyz = origin * position_vec_coords
    oriented_xyz_coord = oriented_xyz.get_pose_vector()

    coordinates = oriented_xyz_coord
    next_pose = coordinates
    robot.set_realtime_pose(next_pose)
    
    return prev_robot_pos

def init_pose():
    global robot_position
    global before_positions
    global first_time
    robot.movej(q=(math.radians(0),
                   math.radians(-53),
                   math.radians(-83),
                   math.radians(-45),
                   math.radians(88),
                   math.radians(0)), a= ACCELERATION , v = VELOCITY)
    robot_position = [0,0,0]
    before_positions = None
    first_time = True
    time.sleep(1)
    #robot.init_realtime_control()
    writeSerial(stdata)
    print('finish init')

def standing_check(x_que,y_que,face_pos):
    #print('standing in')
    x_que.append(abs(face_pos[0]))
    y_que.append(abs(face_pos[1]))
    if len(x_que) == WAITTIME:
        tmp = False
        for i in range(int(WAITTIME/2),WAITTIME):
            if x_que[int(WAITTIME/2)-1] == x_que[i]:
                tmp = True
            else:
                tmp = False
                break
            
        if tmp == True:
        
            init_pose()
            x_que.clear()
            y_que.clear()
            return False,x_que,y_que

        else:
            if sum(x_que)/WAITTIME < 50 and sum(y_que)/WAITTIME < 50:
                if once:
                    temp,distance = distance_check()
                    if distance > 700 :
                        print(temp)
                        if 35.4 < temp < 35.8 :
                            guide(tmp355)
                            guide(qr_start)
                        elif 35.7 < temp < 36.1 :
                            guide(tmp358)
                            guide(qr_start)
                        elif 36.0 < temp < 36.4 :
                            guide(tmp361)
                            guide(qr_start)
                        elif 36.3 < temp < 36.7 :
                            guide(tmp364)
                            guide(qr_start)
                        elif 36.6 < temp < 37.0 :
                            guide(tmp367)
                            guide(qr_start)
                        elif 36.9 < temp < 37.3 :
                            guide(tmp370)
                            guide(qr_start)
                        elif 37.2 < temp < 37.4 :
                            guide(tmp373)
                            guide(qr_start)
                        else:
                            guide(failment)
                            init_pose()
                    else:
                        pass

                #guide(startment)
                    
                    
                return True,x_que,y_que
            else :
                return False,x_que,y_que
    else:
        #print('else')
        return False,x_que,y_que
        
        
def socket_listen():
    HOST = ''
    PORT = 5002
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
    rcvData = client_socket.recv(1024)
    rcvCmd = rcvData.decode()
    data = rcvData.decode('utf-8')
    
    if data == "open":
        robot.set_standard_digital_out(0,True)
        time.sleep(0.3)
        guide(start_effect)
        init_pose()
        robot.set_standard_digital_out(0,False)
    print("data {}".format(data))
   
    print('handler socket close')
    client_socket.close()

def start_sound():
    guide(start_effect)
"""FACE TRACKING LOOP ____________________________________________________________________"""

# initialise robot with URBasic
print("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)

robot.reset_error()
print("robot initialised")
#time.sleep(9)
time.sleep(2)

# Move Robot to the midpoint of the lookplane
init_pose()

robot_position = [0 ,0 ,0]
#origin = set_lookorigin()
#distance_check()
th1 = Thread(target = socket_listen)
th1.start()
#robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised

writeSerial(stdata)

# Display socket
dsp_sock = socket.socket()
dsp_sock.connect((DSP_IP,DSP_PORT))
try:
    print("starting loop")
    while True:
        frame = vs.read()
        face_positions , new_frame = find_faces_dnn(frame)
        show_frame(new_frame)
        #print('face_positions',face_positions)
        if face_positions == None:
            pass
        else:
            if first_time:
                guide_thread = Thread(target=start_sound)
                guide_thread.start()
                writeSerial(rddata)
                stopCheck,xCoord,yCoord = standing_check(xCoord, yCoord, face_positions[0])
                robot_position = move_to_face(face_positions,robot_position)
                first_time = False
            else:
                stopCheck,xCoord,yCoord = standing_check(xCoord, yCoord, face_positions[0])
                robot_position = move_to_face(face_positions,robot_position)

    print("exiting loop")
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()
    if sport.is_open:
        sport.close()

except:
    robot.close()
    if sport.is_open:
        sport.close()
    print('error close')
   








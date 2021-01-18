"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""
from collections import deque
from mpg123 import Mpg123 , Out123
import URBasic
import math
import pickle
import numpy as np
import sys
#import cv2
import time
import serial
import math3d as m3d
import socket
from threading import Timer ,Thread

"""SETTINGS AND VARIABLES ________________________________________________________________"""

ROBOT_IP = '192.168.0.169'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

m_per_pixel = 00.0000062  # Variable which scales the robot movement from pixels to meters.
f_per_pixel = 00.000005

startment = '/home/pi/work/voice/startment.mp3'
beginment = '/home/pi/work/voice/begintemp.mp3'
max_x = 0.2    #Area X
max_y = 0.2    #Area Y
max_z = 0.2    #Area Z
hor_rot_max = math.radians(70)   #Area Camera X
vert_rot_max = math.radians(35)  #Area Camera Y
vector_rot_max = math.radians(15)  #Area Camear Z

sdata = bytearray((0xdc,0x03,0x00,0x00,0x00,0x04)) #sysid, read, dumy ,dumy ,dumy ,read cnt,not include CRC,
sport = serial.Serial("/dev/ttyUSB0", baudrate = 19200, timeout =3.0)
face_distance = 400


start_check = True
first_front_move = False
dc = 0
WAITTIME = 300
xCoord = deque(maxlen = WAITTIME)
yCoord = deque(maxlen = WAITTIME)
time.sleep(0.2)

if len(sys.argv) > 1 and sys.argv[1] == "-l":
    _debug = True
else:
    _debug = False

if _debug:
    print('변수생성 완료')

"""FUNCTIONS _____________________________________________________________________________"""
"""
      Finish Robot move
   """
def guide(ment):

    global start_check
    start_check = False
    mp3 = Mpg123(ment)
    out = Out123()
    
    for frame in mp3.iter_frames(out.start):
        out.play(frame)
    

def distance_check():
    while True:
        global dc
        #global hor_rot_max
        #global vert_rot_max
        #global m_per_pixel
        recvList = []
        sport.write(sdata)
        received_data = sport.read()
        time.sleep(0.03)
        data_left = sport.inWaiting()
        received_data += sport.read(data_left)
        received_data = received_data.decode('ascii')
        save = received_data.split(",")
        for i in save:
            recvList.append(int(i))

        dc = recvList[1]
        #if dc > 350:
        #    m_per_pixel = 00.00001
        #else : 
        #    m_per_pixel = 00.000035
        #print(dc)


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
    if first_front_move:
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
    if first_front_move:
        scaled_face_pos = [face_from_center[0] * m_per_pixel,face_from_center[1] * m_per_pixel,face_from_center[2] * f_per_pixel]
    else:
        scaled_face_pos = [face_from_center[0] * m_per_pixel,face_from_center[1] * m_per_pixel ]
    #print('after',scaled_face_pos)

    if _debug:
        print('second_line')
    #print('zip_before',scaled_face_pos)
    robot_target_xy = [a + b for a, b in zip(prev_robot_pos, scaled_face_pos)]
    #print('zip_after',robot_target_xy)

    robot_target_xy = check_max_xy(robot_target_xy)
    prev_robot_pos = robot_target_xy

    if _debug:
        print('third_line')
    x = robot_target_xy[0]
    y = robot_target_xy[1]
    if  first_front_move:
        z = robot_target_xy[2]    
        
    else:
        z = 0
            
    #print('x,y,z',x,y,z)
    xyz_coords = m3d.Vector(x, y, z)

    x_pos_perc = x / max_x
    y_pos_perc = y / max_y
     
    x_rot = x_pos_perc * hor_rot_max
    y_rot = y_pos_perc * vert_rot_max * -1
     

    tcp_rotation_rpy = [y_rot, x_rot, 0]
    # tcp_rotation_rvec = convert_rpy(tcp_rotation_rpy)
    tcp_orient = m3d.Orientation.new_euler(tcp_rotation_rpy, encoding='xyz')
     
    position_vec_coords = m3d.Transform(tcp_orient, xyz_coords)

    oriented_xyz = origin * position_vec_coords
    oriented_xyz_coord = oriented_xyz.get_pose_vector()

    coordinates = oriented_xyz_coord
    #qnear = robot.get_actual_joint_positions()
    next_pose = coordinates
    robot.set_realtime_pose(next_pose)
    
    return prev_robot_pos

def init_pose():
    global fisrt_front_move
    global robot_position
    robot.movej(q=(math.radians(0),
                   math.radians(-63),
                   math.radians(-93),
                   math.radians(-20),
                   math.radians(88),
                   math.radians(0)), a= ACCELERATION , v = VELOCITY)
    first_front_move = False
    robot_position = [0,0]
    time.sleep(2)
    robot.init_realtime_control()

def standing_check(x_que,y_que,face_pos,first_move_check):
    x_que.append(abs(face_pos[0]))
    y_que.append(abs(face_pos[1]))
    if len(x_que) == WAITTIME:
        print('33')
        if int(sum(x_que[int(len(WAITTIME/2)):])/len(x_que[int(WAITTIME/2):])) == x_que[WAITTIME -1]: 
            print('11')
            init_pose()
            print('22')

        else:
            if sum(x_que)/WAITTIME < 50 and sum(y_que)/WAITTIME < 50:
                if first_move_check:
                    pass
                else:
                    first_move_check = True
                return True,x_que,y_que,first_move_check
            else :
                return False,x_que,y_que,first_move_check
    else:
        print('else')
        return False,x_que,y_que,first_move_check



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
robot.movej(q=(math.radians(0),
               math.radians(-63),
               math.radians(-93),
               math.radians(-20),
               math.radians(88),
               math.radians(0)), a= ACCELERATION, v= VELOCITY )

robot_position = [0,0]
origin = set_lookorigin()

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised
try:
    th = Thread(target = distance_check)
    th.start()
    #Socket Listening 
    print("starting loop")
    HOST = ''
    PORT = 7777
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server_socket.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    print('도대체왜')
    server_socket.bind((HOST,PORT))
    server_socket.listen(200)
    client_socket,addr = server_socket.accept()

    print('connected by',addr)
    while True:
        data = client_socket.recv(4096) 
        face_positions = pickle.loads(data)  #Receive list[] data
        if _debug:
            print('온값',face_positions)
        if face_positions is None or face_positions == None:
            print('NONE값으로 왔음')
        else:
            print('face_position',face_positions)
            stopCheck,xCoord,yCoord,first_front_move = standing_check(xCoord,yCoord,face_positions[0],first_front_move)
            robot_position = move_to_face(face_positions,robot_position)


    print("exiting loop")
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()

except:
    robot.close()
    print('error close')
    

"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""
#______MEMEO__________ : set detection distance change short__________________________#

from collections import deque
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
from threading import Timer ,Thread

"""SETTINGS AND VARIABLES ________________________________________________________________"""

ROBOT_IP = '192.168.0.169'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

DSP_IP = "192.168.0.38"
DSP_PORT = 5001

m_per_pixel = 00.000012  # Variable which scales the robot movement from pixels to meters.  00002
f_per_pixel = 00.000045  #00,000025


max_x = 0.15   #Area X   0.15
max_y = 0.08    #Area Y   0.08
max_z = 0.2    #Area Z    0.2
hor_rot_max = math.radians(35)   #Area Camera X     #35
vert_rot_max = math.radians(20)  #Area Camera Y     #20
vector_rot_max = math.radians(15)  #Area Camear Z


start_check = True
first_time = True
once = True
stun = False
face_positions = ""
WAITTIME = 20
xCoord = deque(maxlen = WAITTIME)
yCoord = deque(maxlen = WAITTIME)
time.sleep(2)

if len(sys.argv) > 1 and sys.argv[1] == "-l":
    _debug = True
else:
    _debug = False


"""FUNCTIONS _____________________________________________________________________________"""
"""
      Finish Robot move
   """
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
    global once
    global stun
    global face_positions
    robot.movej(q=(math.radians(0),
                   math.radians(-53),
                   math.radians(-83),
                   math.radians(-45),
                   math.radians(88),
                   math.radians(0)), a= ACCELERATION , v = VELOCITY)
    robot_position = [0,0,0]
    before_positions = ""
    first_time = True
    once = True
    stun = False
    time.sleep(1)
    face_positions = ""
    robot.init_realtime_control()

def standing_check(x_que,y_que,face_pos):
    global once
    global stun
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
                    print("once")

                else:  #once
                    count =0
                    while True:
                        if count == 10:
                            init_pose()
                            break
                        else:
                            count += 1
                        if stun == True:
                            time.sleep(1)
                        else:
                            init_pose()
                            break
            return True,x_que,y_que
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
    
    if once == False:

        if data == "open":
            robot.set_standard_digital_out(0,True)
            time.sleep(0.3)
            init_pose()
            robot.set_standard_digital_out(0,False)

        print("data {}".format(data))
   
        print('handler socket close')
    else:
        pass
    client_socket.close()


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

robot_position = [0,0,0]
origin = set_lookorigin()
th1 = Thread(target = socket_listen)
th1.start()
print("next start")
robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised


# Display socket
try:
    HOST = ''
    PORT = 5005
    server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
    server_socket.bind((HOST,PORT))
    server_socket.listen(5)
    client_socket, addr = server_socket.accept()
    print("connected by",addr)
    print("starting loop")
    while True:
        data = client_socket.recv(4096)
        face_positions = pickle.loads(data)
        #print('face_positions',face_positions)
        if len(face_positions) == 0:
            print("pass")
            pass
        else:
            if first_time:
                print("first_time")
                stopCheck,xCoord,yCoord = standing_check(xCoord,yCoord,face_positions[0])
                robot_position = move_to_face(face_positions,robot_position)
                first_time = False
            else:
                stopCheck,xCoord,yCoord = standing_check(xCoord,yCoord,face_positions[0])
                if len(face_positions) == 0:
                    pass
                else :
                    robot_position = move_to_face(face_positions,robot_position)

    print("exiting loop")
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()

except:
    robot.close()
    print('error close')
   








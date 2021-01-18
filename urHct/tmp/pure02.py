import os, sys, time 
import serial, socket, logging
import cv2, codecs, traceback
import URBasic
import math, json
import pickle
import numpy as np
import math3d as m3d

from threading import Timer ,Thread
from collections import deque

#------------ System logging Setting ----------------------------
ctrlogger = logging.getLogger("rbctrl")
ctrlogger.setLevel(logging.DEBUG)

stream_hander = logging.StreamHandler()
ctrlogger.addHandler(stream_hander)

#file_handler = logging.FileHandler('robot.log')
#ctrlogger.addHandler(file_handler)

#------------SETTINGS AND VARIABLES -------------------------
if os.path.isfile('rbctrlcfg.dat'):
    with open('rbctrlcfg.dat', 'r') as jfp:
        cfgdat=json.load(jfp)
else:
    rblogger.error("rbctrlcfg.dat not exist!! program stop!!")
    exit()

"""SETTINGS AND VARIABLES ________________________________________________________________"""

ROBOT_IP = cfgdat['ROBOT_IP']
CAM_HOST = cfgdat['CAM_HOST'] 
CAM_PORT = cfgdat['CAM_PORT'] # Camera control host
CTR_HOST = cfgdat['CTR_HOST'] # Voice, Temp meter arm control host
CTR_PORT = cfgdat['CTR_PORT'] # Voice, Temp meter arm control port

ctrlogger.debug("ROBOT_IP : {}".format(ROBOT_IP))
ctrlogger.debug("CAM_PORT : {}".format(CAM_PORT))

ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

m_per_pixel = 00.000025  #   2          Variable which scales the robot movement from pixels to meters.
f_per_pixel = 00.00006  #   2

counter = 0
TIME_OUT = 300
max_x = 0.15   #Area X
max_y = 0.08    #Area Y
max_z = 0.2    #Area Z
hor_rot_max = math.radians(35)   #Area Camera X
vert_rot_max = math.radians(20)  #Area Camera Y
vector_rot_max = math.radians(15)  #Area Camear Z

once = True #Check Loop first time
WAITTIME =40 
xCoord = deque(maxlen = WAITTIME)
yCoord = deque(maxlen = WAITTIME)
time.sleep(2)

emissivity = float(cfgdat['emissiv'])

def temp_Voice(ctr_host, ctr_port, mentVal):
    tv_Socket = socket.socket()
    tv_Socket.connect((ctr_host, int(ctr_port)))
 
    mentVal = int(mentVal / emissivity)
    mentVal = int((mentVal - 3000) / 10) # make range 70 to  31
    
    if mentVal > 70:
        mentVal = 70
    elif mentVal < 31:
        mentVal = 31
    
    cmdList = [ 0xff, 0xff, 0xdc, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 ]
    cmdList[9] = mentVal
    
    cmdStr = bytes(cmdList).hex()
    tv_Socket.sendall(bytes(cmdStr.encode("utf-8")))
    data = codecs.decode(tv_Socket.recv(1024))
    
def move_Cam(ctr_host, ctr_port, direction):
    tv_Socket = socket.socket()
    tv_Socket.connect((ctr_host, int(ctr_port)))

    cmdList = [ 0xff, 0xff, 0xdc, 0x04, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00 ]
    
    if direction == "front":
        cmdList[4] = 1 # temp sensor front
        cmdList[5] = 1 # wing open
        cmdList[6] = 1 # smail eye
        cmdList[8] = 3 # blue LED on
        cmdList[9] = 1 # Start_sound
    elif direction == "back":
        cmdList[4] = 2 # temp sensor back
        cmdList[5] = 2 # wing close
        cmdList[6] = 0 # sleep eye
        cmdList[8] = 2 # green LED on
    
    cmdStr = bytes(cmdList).hex()
    tv_Socket.sendall(bytes(cmdStr.encode("utf-8")))
    data = codecs.decode(tv_Socket.recv(1024))

def check_max_xy(xy_coord):
    x_y = [0,0,0]

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
    if  0 <= xy_coord[2] <= max_z:
        # checks if the resulting position would be outside of max_y
        x_y[2] = xy_coord[2]
    elif  0 > xy_coord[2]:
        x_y[2] =  0  
    elif max_z < xy_coord[2]:
        x_y[2] = max_z
    else:
        raise Exception(" y is wrong somehow", xy_coord[1], max_y)
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
    
    face_from_center = list(list_of_facepos[0])  # TODO: find way of making the selected face persistent

    prev_robot_pos = robot_pos
    
    scaled_face_pos = [face_from_center[0] * m_per_pixel,face_from_center[1] * m_per_pixel,face_from_center[2] * f_per_pixel]
    

    robot_target_xy = [a + b for a, b in zip(prev_robot_pos, scaled_face_pos)]

    robot_target_xy = check_max_xy(robot_target_xy)
    prev_robot_pos = robot_target_xy

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
    global once
    global xCoord
    robot.movej(q=(math.radians(0),
                   math.radians(-63),
                   math.radians(-93),
                   math.radians(-20),
                   math.radians(88),
                   math.radians(0)), a= ACCELERATION , v = VELOCITY)
    robot_position = [0,0,0]
    once = True
    time.sleep(2)
    robot.init_realtime_control()
    ctrlogger.debug('finish init')

def is_Move(xCoord):
    for i in range(len(xCoord)):
        if xCoord[0] == xCoord[i]:
            tmp = False
        else:
            tmp = True
            break
    return tmp
    
def compare_joint():
    joint = robot.get_actual_joint_positions()
    joint = np.round(joint,2)
    joint = joint.tolist()
    joint.sort()
    if joint[0] == init_radians[0] and joint[1] == init_radians[1] and joint[2] == init_radians[2]:
        pass
    else:
        init_pose()

def reset_timer():
    global counter
    while True:
        if counter > 20:
            pass

        if counter == 20:
            move_Cam(CTR_HOST, CTR_PORT, "back")
        else:
            counter += 1
            time.sleep(1)


"""FACE TRACKING LOOP ____________________________________________________________________"""

# initialise robot with URBasic
ctrlogger.info("initialising robot")
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)

robot.reset_error()
ctrlogger.info("robot initialised")
#time.sleep(9)
time.sleep(2)

# Move Robot to the midpoint of the lookplane
init_pose()
ctrlogger.debug("Thread init")
robot_position = [0, 0, 0]
origin = set_lookorigin()
init_radians = robot.get_actual_joint_positions()
init_radians = np.round(init_radians,2)
init_radians = init_radians.tolist()
init_radians.sort()

robot.init_realtime_control()  # starts the realtime control loop on the Universal-Robot Controller
time.sleep(1) # just a short wait to make sure everything is initialised
try:
    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    client_socket.connect((CAM_HOST, int(CAM_PORT)))
    time_th = Thread(target=reset_timer)
    time_th.start()
    while True:
        data = client_socket.recv(4096)
        #ctrlogger.debug("pos {}".format(face_positions)) 
        if len(data) == 0:
            pass
        else:
            face_positions = pickle.loads(data)
            face_pose = face_positions[0]
            xCoord.append(face_pose[0])
            yCoord.append(face_pose[1])
            move = True
            if len(xCoord) > 5:
                move = is_Move(xCoord)
                if move == False:
                    compare_joint()
                else:  # move == True
                    counter = 0
                    if once == True:
                        ctrlogger.debug("enter once")
                        cam_move_th = Thread(target=move_Cam, args=(CTR_HOST, CTR_PORT, "front"))
                        cam_move_th.daemon = True
                        cam_move_th.start()
                        #move_Cam(CTR_HOST, CTR_PORT, "front")
                        ctrlogger.debug("after move_cam")
                        once = False
                        robot_position = move_to_face(face_positions,robot_position)
                    else:
                        ddis = int(face_pose[3])
                        ttemp = float(face_pose[4])
                        if abs(face_pose[0]) < 20 and abs(face_pose[1]) < 20 and ddis < 100 :
                            temperature = (ttemp / emissivity) 
                            temperature = round(temperature,1)
                            temp_Voice(CTR_HOST, CTR_PORT, ttemp)
                            ctrlogger.info("temperature : {}".format(temperature))
                            client_socket.close()
                            init_pose()
                            time.sleep(0.5)
                            connected = False
                            while not connected:
                                try:
                                    client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
                                    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                                    client_socket.connect((CAM_HOST, int(CAM_PORT)))
                                    ctrlogger.debug("Try reconnect")
                                    connected = True
                                except socket.error:
                                    time.sleep(1)
                        else:
                            robot_position = move_to_face(face_positions,robot_position)
            else:
                pass

    ctrlogger.debug("exiting loop")
except KeyboardInterrupt:
    ctrlogger.error("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()

except Exception as e:
    print(str(e))
    logging.error(traceback.format_exc())
    robot.close()
    ctrlogger.error('error close')
   








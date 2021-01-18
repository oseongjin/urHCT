"""
Face_tracking01
Python program for realtime face tracking of a Universal Robot (tested with UR5cb)
see here for a demonstration: https://youtu.be/HHb-5dZoPFQ

Created by Robin Godwyll
License: GPL v3 https://www.gnu.org/licenses/gpl-3.0.en.html


"""

import URBasic
import math
import numpy as np
import sys
import cv2
import time
import imutils
from imutils.video import VideoStream
import math3d as m3d

"""SETTINGS AND VARIABLES ________________________________________________________________"""

RASPBERRY_BOOL = False
# If this is run on a linux system, a picamera will be used.
# If you are using a linux system, with a webcam instead of a raspberry pi delete the following if-statement
if sys.platform == "linux":
    import picamera
    from picamera.array import PiRGBArray
    RASPBERRY_BOOL = True

ROBOT_IP = '192.168.0.169'
ACCELERATION = 0.9  # Robot acceleration value
VELOCITY = 0.8  # Robot speed value

# Path to the face-detection model:
#pretrained_model = cv2.dnn.readNetFromCaffe("MODELS/deploy.prototxt.txt", "MODELS/res10_300x300_ssd_iter_140000.caffemodel")
pretrained_model = cv2.dnn.readNet('models/face-detection-adas-0001.xml','models/face-detection-adas-0001.bin')
pretrained_model.setPreferableTarget(cv2.dnn.DNN_TARGET_MYRIAD)

video_resolution = (1024, 768)  # resolution the video capture will be resized to, smaller sizes can speed up detection
video_midpoint = (int(video_resolution[0]/2),
                  int(video_resolution[1]/2))
video_asp_ratio  = video_resolution[0] / video_resolution[1]  # Aspect ration of each frame
video_viewangle_hor = math.radians(25)  # Camera FOV (field of fiew) angle in radians in horizontal direction


m_per_pixel = 00.0000062  # Variable which scales the robot movement from pixels to meters.
f_per_pixel = 00.000005

max_x = 0.2
max_y = 0.2
hor_rot_max = math.radians(70)
vert_rot_max = math.radians(35)

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
vs = VideoStream(src = 0,
                 usePiCamera = True,
                 resolution=video_resolution).start()
time.sleep(0.2)



"""FUNCTIONS _____________________________________________________________________________"""

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
    temp = 0
    save = []

    blob = cv2.dnn.blobFromImage(frame,size = (300,300),ddepth = cv2.CV_8U)
    pretrained_model.setInput(blob)

    detections = pretrained_model.forward()
    face_centers = []
    # loop over the detections
    for s in detections.reshape(-1,7):
        # extract the confidence (i.e., probability) associated with the prediction
        confidence = float(s[2])
        fstartX = int(s[3] * frame.shape[1])
        fstartY = int(s[4] * frame.shape[0])
        fendX = int(s[5] * frame.shape[1])
        fendY = int(s[6] * frame.shape[0])


        if confidence < 0.8:
            continue

        else:
            pred_boxpts = (fstartX,fstartY,fendX,fendY)
            save.append(pred_boxpts)
    print('save',save)

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
        if first_max - second_max < 50:
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
        text = "{:.2f}%".format(confidence * 100)
        y = startY - 10 if startY - 10 > 10 else startY + 10
        face_center = (int(startX + (endX - startX) / 2), int(startY + (endY - startY) / 2))
        dis = 300 - (endX -startX)
        position_from_center = (face_center[0] - video_midpoint[0], face_center[1] - video_midpoint[1],dis)
        if endX - startX < 90:
            return 0
        face_centers.append(position_from_center)

        cv2.rectangle(frame, (startX, startY), (endX, endY),
                      (0, 0, 255), 2)
        cv2.putText(frame, text, (startX, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
        #cv2.putText(frame, str(position_from_center), face_center, 0, 1, (0, 200, 0))
        cv2.line(frame, video_midpoint, face_center, (0, 200, 0), 5)
        cv2.circle(frame, face_center, 4, (0, 200, 0), 3)

    return face_centers, frame

def show_frame(frame):
    cv2.imshow('img', frame)
    k = cv2.waitKey(6) & 0xff


def check_max_xy(xy_coord):
    ''' 
    Checks if the face is outside of the predefined maximum values on the lookaraound plane

    Inputs:
        xy_coord: list of 2 values: x and y value of the face in the lookaround plane.
            These values will be evaluated against max_x and max_y

    Return Value:
        x_y: new x and y values
            if the values were within the maximum values (max_x and max_y) these are the same as the input.
            if one or both of the input values were over the maximum, the maximum will be returned instead
     
    '''

    x_y = [0, 0,0]
    #print("xy before conversion: ", xy_coord)

    if -max_x <= xy_coord[0] <= max_x:
        x_y[0] = xy_coord[0]
    elif -max_x > xy_coord[0]:
        x_y[0] = -max_x
    elif max_x < xy_coord[0]:
        x_y[0] = max_x
    else:
        raise Exception(" x is wrong somehow:", xy_coord[0], -max_x, max_x)

    if -max_y <= xy_coord[1] <= max_y:
        x_y[1] = xy_coord[1]
    elif -max_y > xy_coord[1]:
        x_y[1] = -max_y
    elif max_y < xy_coord[1]:
        x_y[1] = max_y
    else:
        raise Exception(" y is wrong somehow", xy_coord[1], max_y)
    
    if first_front_move:
        if 0 <= xy_coord[2] <= max_y:
            x_y[2] = xy_coord[2]
        elif 0 > xy_coord[2]:
            x_y[2] = 0
        elif max_y < xy_coord[2]:
            x_y[2] = max_y
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


    face_from_center = list(list_of_facepos[0])  # TODO: find way of making the selected face persistent

    prev_robot_pos = robot_pos
    if first_front_move:
        scaled_face_pos = [face_from_center[0] * m_per_pixel, face_from_center[1] * m_per_pixel , face_from_center[2] * f_per_pixel]
    else:
        scaled_face_pos = [face_from_center[0] * m_per_pixel, face_from_center[1] * m_per_pixel]


    robot_target_xy = [a + b for a, b in zip(prev_robot_pos, scaled_face_pos)]
    # print("..", robot_target_xy)

    robot_target_xy = check_max_xy(robot_target_xy)
    prev_robot_pos = robot_target_xy

    x = robot_target_xy[0]
    y = robot_target_xy[1]
    if first_front_move:
        z = robot_target_xy[2]
    else:
        z = 0
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

    qnear = robot.get_actual_joint_positions()
    next_pose = coordinates
    robot.set_realtime_pose(next_pose)

    return prev_robot_pos

def init_pose():
    global first_front_move
    global robot_position
    robot.movej(q=(math.radians(0),
                   math.radians(-63),
                   math.radians(-93),
                   math.radians(-20),
                   math.radians(88),
                   math.radians(0)), a = ACCELERATION , v = VELOCITY)
    first_front_move = False
    robot_position = [0,0]
    time.sleep(2)
    robot.init_realtime_control()


def standing_check(x_que,y_que,face_pos,first_move_check):
    x_que.append(abs(face_pos[0]))
    y_que.append(abs(face_pos[1]))
    if len(x_que) == WAITTIME:
        if int(sum(x_que[int(len(WAITTIME/2)):])/len(x_que[int(WAITTIME/2):])) == x_que[WAITTIME -1 ]:
            init_pose()
        else:
            if sum(x_que)/WAITTIME < 50 and sum(y_que)/WAITTIME < 50:
                if first_move_check:
                    pass
                else:
                    first_move_check = True
                return True,x_que,y_que,first_move_check

            else:
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
time.sleep(1)

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
    print("starting loop")
    while True:

        frame = vs.read()
        face_positions, new_frame = find_faces_dnn(frame)
        show_frame(new_frame)
        if len(face_positions) > 0:
            robot_position = move_to_face(face_positions,robot_position)

    print("exiting loop")
except KeyboardInterrupt:
    print("closing robot connection")
    # Remember to always close the robot connection, otherwise it is not possible to reconnect
    robot.close()

except:
    robot.close()

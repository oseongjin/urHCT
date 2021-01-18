import URBasic
import math
ROBOT_IP = "192.168.0.169"
ACCELERATION = 0.9
VELOCITY = 0.8

robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host = ROBOT_IP,robotModel = robotModel)

while True:
    robot.movej(q=(math.radians(0),
                   math.radians(-53),  #add back
                   math.radians(-83),  #83
                   math.radians(-45),  #add high
                   math.radians(88),
                   math.radians(0)), a = ACCELERATION , v = VELOCITY)

    robot.movej(q=(math.radians(0),
                   math.radians(-53),
                   math.radians(-83),
                   math.radians(-30),
                   math.radians(88),
                   math.radians(0)), a = ACCELERATION , v = VELOCITY)

# ---original pose---
robot.movej(q=(math.radians(0),
               math.radians(-63),
               math.radians(-93),
               math.radians(-20),
               math.radians(88),
               math.radians(0)), a = ACCELERATION , v = VELOCITY)


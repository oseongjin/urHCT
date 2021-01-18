import URBasic
import time
ROBOT_IP = "192.168.0.169"

robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host=ROBOT_IP,robotModel=robotModel)
dashboardClient = URBasic.dashboard.DashBoard(robotModel)
dashboardClient.ur_popup("hello world")
time.sleep(3)
dashboardClient.ur_close_popup()
while True:
    time.sleep(1)
    #state = robot.robotConnector.RobotModel.SafetyStatus().NormalMode
    state = robot.robotConnector.RobotModel.RobotStatus().PowerOn
    #ursafe = dashboardClient.ur_safetymode()
    print(state)

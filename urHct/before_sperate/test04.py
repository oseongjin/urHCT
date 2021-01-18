import URBasic

ROBOT_IP = "192.168.0.169"
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScriptExt.UrScriptExt(host = ROBOT_IP,robotModel=robotModel)


time.sleep(2)
while True:
    print("hello")

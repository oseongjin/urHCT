import URBasic


ROBOT_IP = '192.168.0.169'
robotModel = URBasic.robotModel.RobotModel()
robot = URBasic.urScript.UrScript(host=ROBOT_IP,robotModel=robotModel)

robot.set_standard_digital_out(0, True)

print('close')
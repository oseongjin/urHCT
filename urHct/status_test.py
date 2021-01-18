
while True:
    robot_status = robot.reset_error()
    if robot_status == True:
        break
    else:
        time.sleep(3)

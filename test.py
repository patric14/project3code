import robot_team17
robot = robot_team17.RobotLibrary()
'''
speed = robot.getval('Input speed: ')
direction = robot.getval('direction: ')
'''
try:

    robot.turn(0, 90, 150)

except KeyboardInterrupt:
    pass

robot.kill()
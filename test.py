import robot_team17
import time
import brickpi3
robot = robot_team17.RobotLibrary()
BP = brickpi3.BrickPi3()

try:
    robot.fix_arm()
    print('arm fixed')
    print('Weight: ', robot.weigh())
except KeyboardInterrupt:
    pass

robot.kill()

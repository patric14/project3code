import robot_team17
import time
import brickpi3
robot = robot_team17.RobotLibrary()
BP = brickpi3.BrickPi3()

try:
    while True:
        robot.drive_dist(1, 40)
except KeyboardInterrupt:
    pass

robot.kill()

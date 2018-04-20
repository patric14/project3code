import robot_team17
import time
robot = robot_team17.RobotLibrary()

try:
    robot.drive_dist(1, 15, 40)
except KeyboardInterrupt:
    pass

robot.kill()

import robot_team17
import time
robot = robot_team17.RobotLibrary()

try:
    print('Magnet: ', robot.magnet())
except KeyboardInterrupt:
    pass

robot.kill()

import robot_team17
import time
robot = robot_team17.RobotLibrary()

try:
    while True:
        robot.magnet()
        time.sleep(1)
except KeyboardInterrupt:
    pass

robot.kill()

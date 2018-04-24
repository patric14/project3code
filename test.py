import meme
import time
import brickpi3
robot = meme.RobotLibrary()
BP = brickpi3.BrickPi3()

try:
    robot.check_junction(30)

except KeyboardInterrupt:
    pass

robot.kill()

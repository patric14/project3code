import meme
import time
import brickpi3
robot = meme.RobotLibrary()
BP = brickpi3.BrickPi3()

try:
<<<<<<< HEAD
    robot.check_junction(30)

=======
    while True:
        robot.drive_dist(1, 40)
>>>>>>> 9ad743a2abee20f476b3903fc31d2c517849e986
except KeyboardInterrupt:
    pass

robot.kill()

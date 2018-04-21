import robot_team17
import time
import brickpi3
robot = robot_team17.RobotLibrary()
BP = brickpi3.BrickPi3()

try:
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))
    while True:
        print(BP.get_motor_encoder(BP.PORT_C))
        time.sleep(.5)
except KeyboardInterrupt:
    pass

robot.kill()

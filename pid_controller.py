import robot_team17
import time
import brickpi3

robot = robot_team17.RobotLibrary()
BP = brickpi3.BrickPi3

TARGET = 5
dT = .2
KP = robot.getval('Input kP: ')

power = 50

try:
    while True:
        currentDist = robot.check_distance()

        e = TARGET - currentDist

        P = -1 * KP * e

        BP.set_motor_power(BP.PORT_D, power - P, power - P)
        BP.set_motor_power(BP.PORT_A, power + P, power + P)

        time.sleep(dT)
except KeyboardInterrupt:
    pass

robot.kill()

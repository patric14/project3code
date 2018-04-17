import robot_team17
import time
import brickpi3
from math import atan, cos

robot = robot_team17.RobotLibrary()
BP = brickpi3.BrickPi3()

dT = robot.getval('dT: ')
target = robot.getval('Input target distance: ')
KP = robot.getval('Input kP: ')

power = 50

previousDist = robot.check_distance()
positionPreviousLeft = BP.get_motor_encoder(BP.PORT_D)
positionPreviousRight = BP.get_motor_encoder(BP.PORT_A)

P = 0

try:
    while True:
        BP.set_motor_power(BP.PORT_D, power + P)
        BP.set_motor_power(BP.PORT_A, power - P)
        
        currentDist = robot.check_distance()
        positionCurrentLeft = BP.get_motor_encoder(BP.PORT_D)
        positionCurrentRight = BP.get_motor_encoder(BP.PORT_A)

        distDiff = currentDist - previousDist
        
        leftDistDrive = robot.DIST_DEG * (positionCurrentLeft - positionPreviousLeft)
        rightDistDrive = robot.DIST_DEG * (positionCurrentRight - positionPreviousRight)
        distDrive = (leftDistDrive + rightDistDrive) / 2
        
        angle = atan(distDiff / distDrive)
        
        distPerp = currentDist * cos(abs(angle))
        
        P = (distPerp - 5)
        K = P / 5
        degrees = (1 + K) * abs(angle)
        
        if (angle > 0) and (P > 0):
            robot.turn(0, degrees, 100)
        if (angle < 0) and (P < 0):
            robot.turn(1, degrees, 100)

        time.sleep(dT)

        previous = currentDist

except KeyboardInterrupt:
    pass

robot.kill()

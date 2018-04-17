import robot_team17
import time
import brickpi3
from math import atan, cos

robot = robot_team17.RobotLibrary()
BP = brickpi3.BrickPi3()

dT = .2 #robot.getval('dT: ')
target = 15 # robot.getval('Input target distance: ')
KP = 5 # robot.getval('Input kP: ')
#KI = robot.getval('Input KI: ')

power = 50

previousDist = robot.check_distance()
positionPreviousLeft = BP.get_motor_encoder(BP.PORT_D)
positionPreviousRight = BP.get_motor_encoder(BP.PORT_A)

P = 0
I = 0


try:
    while True:
        correction = P + I
        BP.set_motor_power(BP.PORT_D, power - correction)
        BP.set_motor_power(BP.PORT_A, power + correction)
        
        currentDist = robot.check_distance()
        positionCurrentLeft = BP.get_motor_encoder(BP.PORT_D)
        positionCurrentRight = BP.get_motor_encoder(BP.PORT_A)

        distDiff = currentDist - previousDist
        
        leftDistDrive = robot.DIST_DEG * abs(positionCurrentLeft - \
                                             positionPreviousLeft)
        rightDistDrive = robot.DIST_DEG * abs(positionCurrentRight - \
                                              positionPreviousRight)
        distDrive = (leftDistDrive + rightDistDrive) / 2
        
        angle = atan(distDiff / distDrive)
        
        distPerp = currentDist * cos(abs(angle))
        
        e = (distPerp - target)
        
        P = KP * e
        K = P / 5
        '''degrees = (1 + K) * abs(angle)
        if (angle > 0) and (P > 0):
            robot.turn(0, degrees, 100)
        if (angle < 0) and (P < 0):
            robot.turn(1, degrees, 100)'''

        time.sleep(dT)

        previous = currentDist

except KeyboardInterrupt:
    pass

robot.kill()

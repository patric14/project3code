import robot_team17
import brickpi3
import time

BP = brickpi3.BrickPi3()
robot = robot_team17.RobotLibrary()

robot.reset_encoder(robot.ULTRASONIC)

def check_ultra_angle():
    return(BP.get_motor_encoder(robot.ULTRASONIC))

def turn_ultrasonic(direction):

    if direction == robot.LEFT:
        print('left')
        while check_ultra_angle() < 160:
            print(check_ultra_angle())
            BP.set_motor_power(robot.ULTRASONIC, 100)
    elif direction == robot.RIGHT:
        print('right')
        while check_ultra_angle() > -160:
            print(check_ultra_angle())
            BP.set_motor_power(robot.ULTRASONIC, -100)
    else:
        while abs(check_ultra_angle()) > 5:
            print(check_ultra_angle())
            BP.set_motor_position(robot.ULTRASONIC, 0)


direction = input('direction: ')

try:
    while check_ultra_angle() > -160:
        print(check_ultra_angle())
        BP.set_motor_power(robot.ULTRASONIC, -100)
except KeyboardInterrupt:
    pass

robot.kill()

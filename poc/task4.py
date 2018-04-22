import robot_team17
import time

robot = robot_team17.RobotLibrary()

try:
    '''xCoord = robot.getval('Input X coordinate: ')
    yCoord = robot.getval('Input Y coordinate: ')
    xCoord = robot.getval('Input X coordinate: ')
    yCoord = robot.getval('Input Y coordinate: ')
    xCoord = robot.getval('Input X coordinate: ')
    yCoord = robot.getval('Input Y coordinate: ')
    xCoord = robot.getval('Input X coordinate: ')
    yCoord = robot.getval('Input Y coordinate: ')
    #block_dist = robot.getval('Input distance between blocks: ')'''
    speed = robot.getval('Input speed: ')


    robot.drive_dist(35, speed)
    robot.stop()
    time.sleep(3)
    robot.drive_dist(35, speed)
    robot.stop()
    time.sleep(.1)
    robot.turn(robot.RIGHT, 90, speed)
    robot.drive_dist(35, speed)
    robot.stop()
    time.sleep(3)
    robot.drive_dist(70, speed)
    robot.stop()
    time.sleep(.1)
    robot.turn(robot.RIGHT, 90, speed)
    robot.stop()
    robot.drive_dist(70, speed)
    robot.stop()
    time.sleep(3)
    robot.turn(robot.LEFT, 180, speed)
    robot.stop()
    time.sleep(.1)
    robot.drive_dist(105, speed)
    robot.stop()
except KeyboardInterrupt:
    pass

robot.kill()

# POC Task 1
# File: task1.py
# Date: 2018
# By: Abby Beard
# beard29
# Caleb Patrick
# patric14
# Haydn Schroader
# hschroad
# Zach McClary
# zmcclary
# Section: 1
#
# Team: Team Number #
# ELECTRONIC SIGNATURE
# Abby Beard
# Caleb Patrick
# Haydn Schroader
# Zach McClary
#
# The electronic signatures above indicate that the program
# submitted for evaluation is the combined effort of all
# team members and that each member of the team was an
# equal participant in its creation. In addition, each
# member of the team has a general understanding of
# all aspects of the program development and execution.
#
# Description

import robot_team17
import time

robot = robot_team17.RobotLibrary()

try:

    print('POC Task 1: Navigation through hallway.')
    block_size = robot.getval('Block Size: ')
    #time_limit = robot.getval('Time limit: ')
    speed = robot.getval('Input speed: ')
    dist = block_size + 1

    while True:

        while dist > block_size:
            dist = robot.check_distance()
            robot.drive(speed)
        robot.stop()
        robot.drive_dist(block_size / 2, speed)
        
        numJunction, typeJunction = robot.check_junction(block_size)

        if (typeJunction == robot.JUNCT_LEFT) | \
        (typeJunction == robot.JUNCT_LEFT_RIGHT) | \
        (typeJunction == robot.JUNCT_LEFT_STRAIGHT):
            robot.turn(robot.LEFT, 90, speed)
        elif (typeJunction == robot.JUNCT_RIGHT) | \
        (typeJunction == robot.JUNCT_RIGHT_STRAIGHT):
            robot.turn(robot.RIGHT, 90, speed)

except KeyboardInterrupt:
    pass

robot.kill()

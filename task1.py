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

    while True:

        junction = robot.check_junction(block_size)

        if junction == robot.LEFT:
            robot.turn(robot.LEFT, 90, speed)
        elif junction == robot.RIGHT:
            robot.turn(robot.RIGHT, 90, speed)

        robot.drive_dist(block_size / 2, speed)

except KeyboardInterrupt:
    pass

robot.kill()

# POC Task 2
# File: task2.py
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
robot = robot_team17.RobotLibrary()

try:

    print('POC Task 2: Turn to angle')
    degrees = float(input('Input degrees: '))
    power = 100  # float(input('Input motor power: '))
    direction = float(input('Input 0 for left turn, 1 for right turn: '))

    #robot.button()

    robot.turn(direction, degrees, power)

except KeyboardInterrupt:
    pass

robot.kill()

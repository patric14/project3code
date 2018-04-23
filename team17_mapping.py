# Project 3
# File: team17_mapping.py
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
# This script runs the mapping routine for the Team 17 robot.

import robot_team17
import time

robot = robot_team17.RobotLibrary()

IN2FT = 12
IN2CM = 2.54
mapLengthX = 22 * IN2FT * IN2CM #cm
mapLengthY = 12 * IN2FT * IN2CM #cm

fully_mapped = False
explore_space = False

map_number, block_size, unit, origin, notes = robot.setup()
'''
map_number = 1
block_size = 30.5
unit = 'cm'
origin = [0, 0]
notes = 'notes'
'''

resources = robot.resourceInfo()

mapBlockX = int(mapLengthX / block_size + 2)
mapBlockY = int(mapLengthY / block_size + 2)

positionX = int(origin[0])
positionY = int(origin[1])

mapMatrix = robot.mapSetup(mapBlockX, mapBlockY)

direction = robot.UP

try:

    direction, positionX, positionY = robot.explore_space(block_size, mapMatrix, direction, positionX,  positionY)

    openY = 0
    openX = 0
    for i in range(len(mapMatrix)):
        current1 = mapMatrix[i]
        for j in range(len(current1)):
            current2 = current1[j]
            if (current2 == 6):
                openY = current1
                openX = current2
                current1 = len(mapMatrix)
                current2 = len(current1)

    if (openY != 0):
        robot.return_open_space()
    robot.map_output(map_number, block_size, unit, origin, notes)
except KeyboardInterrupt:
    print('Program Interrupted')
'''except:
    print('Error')'''

robot.map_output(map_number, block_size, unit, origin, notes, mapMatrix)
robot.resource_output(resources, map_number, notes)

robot.kill()

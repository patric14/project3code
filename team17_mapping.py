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
origin = [5, 5]
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

    direction, positionX, positionY, pastX, pastY = robot.explore_space_simple(block_size, mapMatrix, direction, positionX, positionY)

    direction, positionX, positionY = robot.explore_open(block_size, mapMatrix, direction, positionX, positionY, pastX, pastY)
    robot.map_output(map_number, block_size, unit, origin, notes)
except KeyboardInterrupt:
    print('Program Interrupted')
except:
    print('Error')

robot.map_output(map_number, block_size, unit, origin, notes, mapMatrix)
robot.resource_output(resources, map_number, notes)

robot.kill()

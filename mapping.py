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

map_number, block_size, unit, origin, notes = setup()

mapBlockX = mapLengthX / unit_length + 2
mapBlockY = mapLengthY / unit_length + 2

mapMatrix = robot.mapSetup(mapBlockX, mapBlockY)

direction, positionX, positionY = robot.explore_space(block_size, mapMatrix, \
direction, positionX, positionY)

map_output(map_number, unit_length, unit, origin, notes)

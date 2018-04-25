# Project 3
# File: robot_team17.py
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
# This is the function library for team 17's mapping robot.

# Imports
import brickpi3
import grovepi
from MPU9250 import MPU9250
from math import pi, atan, cos
import time
import numpy as np
import IR_Functions as ir

# Object Creation
BP = brickpi3.BrickPi3()
imu = MPU9250()

# Library class
class RobotLibrary(object):

    # Constants

    TEAM = 17

    # Mapping Constants
    PATH = 1
    NOT_PATH = 0
    ORIGIN = 10
    BIOHAZARD = 2
    NONHAZARDOUS_WASTE = 3
    RADIATION = 4
    MRI = 5
    START = 6
    EXIT = 7

    # Robot hardware constants
    LEFT_MOTOR = BP.PORT_D
    RIGHT_MOTOR = BP.PORT_A
    ULTRASONIC_MOTOR = BP.PORT_B
    ARM_MOTOR = BP.PORT_C
    BUTTON = BP.PORT_1
    LIGHT = BP.PORT_2
    WHEEL_RADIUS = 1.97
    TRACK_SEPARATION = 18
    ULTRASONIC = 2
    ULTRASONIC_RIGHT = 165
    ULTRASONIC_LEFT = 165
    SCANTARGET = 5

    # Direction
    LEFT = 0
    RIGHT = 1
    UP = 2
    DOWN = 3
    UTURN = 4

    # Other Constants
    STRAIGHT = 2
    BIOHAZARD_COLOR = 'Biohazard' # Yellow
    NONHAZARD_COLOR = 'Nonhazardous' # Blues
    WALL_COLOR = 'Wall' # White
    HAZARD_THRESHOLD = 2800
    WALL_THRESHOLD = 2500
    CESIUM_THRESHOLD = 180
    MRI_THRESHOLD = -39
    DIST_DEG = (2 * pi * WHEEL_RADIUS) / 360
    wheel_track_ratio = TRACK_SEPARATION / WHEEL_RADIUS
    FULL_TURN = 1920
    DEG_TURN = FULL_TURN / 360
    POWER = 50
    KP = 4

    # Junction types
    JUNCT_DEAD_END = 0
    JUNCT_STRAIGHT = -1
    JUNCT_LEFT = -10
    JUNCT_RIGHT = -100
    JUNCT_LEFT_RIGHT = -110
    JUNCT_LEFT_STRAIGHT = -11
    JUNCT_RIGHT_STRAIGHT = -101
    JUNCT_ALL_WAY = -111


    # Sensor setup
    BP.set_sensor_type(BUTTON, BP.SENSOR_TYPE.TOUCH)
    BP.set_sensor_type(LIGHT, BP.SENSOR_TYPE.NXT_LIGHT_ON)

    # Functions

    def voltage(self):
        voltage = BP.get_voltage_battery()
        print('Battery Voltage: ', voltage)
        return voltage

    def getval(self, string):
        val = float(input(string))
        return val

    def setup(self):
        # This function sets up all the bad bois

        origin = []
        map_number = self.getval('Input map: ')
        block_size = self.getval('Input unit length: ')
        unit = input('Input unit: ')
        origin.append(self.getval('Input x origin coordinate: '))
        origin.append(self.getval('Input y origin coordinate: '))
        notes = input('Notes: ')

        return map_number, block_size, unit, origin, notes

    def mapSetup(self, xSize, ySize):

        # This function sets up the map matrix

        mapMatrix = [[0 for col in range(ySize)] for row in range(xSize)]

        return mapMatrix

    def resourceInfo(self):

        # This function creates an array to store resource location and
        # information

        header = []
        header.append('Parameter of interest')
        header.append('Parameter')
        header.append('Resource X Coordinate')
        header.append('Resource Y Coordinate')
        biohazard = []
        biohazard.append('Biohazard')
        biohazard.append('Mass(g)')
        for i in range(1, 3):
            biohazard.append(0)
        cesium = []
        cesium.append('Cesium-137')
        cesium.append('Radiation Strength (W)')
        for i in range(1, 3):
            cesium.append(0)
        nonHazardous = []
        nonHazardous.append('Non-Hazardous Waste')
        nonHazardous.append('Mass (g)')
        for i in range(1, 3):
            nonHazardous.append(0)

        resources = [header, biohazard, cesium, nonHazardous]

        return resources

    def button(self):

        # This function waits for the touch sensor to be pressed before the
        # next code section is executed.

        trigger = 0
        print('Press touch sensor to start')

        while trigger == 0:
            try:
                trigger = BP.get_sensor(self.BUTTON)
            except brickpi3.SensorError as error:
                print(error)

    def scanner(self):

        # This function uses the light sensor to determine which type of waste
        # it is looking at

        scan = BP.get_sensor(self.LIGHT)
        print('Sensor value: ', scan)
        if scan > self.HAZARD_THRESHOLD:
            return self.NONHAZARD_COLOR
        elif scan < self.WALL_THRESHOLD:
            return self.WALL_COLOR
        else:
            return self.BIOHAZARD_COLOR

    def ir_read(self):

        # This function reads the IR sensor data

        [sens1, sens2] = ir.IR_Read(grovepi)

        print('IR Reading: ', sens2)

        if sens2 > 180:
            return True
        else:
            return False

    def magnet(self):

        # This function returns the magnetic sensor value

        raw = imu.readMagnet()

        magnet = raw['y']

        print('Magnet Reading: ', magnet)

        if magnet < self.MRI_THRESHOLD:
            return True
        else:
            return False

    def stop(self):
        BP.set_motor_dps(self.LEFT_MOTOR + self.RIGHT_MOTOR + \
                         self.ULTRASONIC_MOTOR, 0)

    def reset_encoder(self, motor):

        BP.offset_motor_encoder(motor, \
                                BP.get_motor_encoder(motor))

    def drive_dist(self, num_blocks, block_size):

        # This function drives the robot at the input speed for the input
        # distance

        if num_blocks * block_size < 0:
            direction = -1
        else:
            direction = 1

        num_blocks = abs(num_blocks)
        block_size = abs(block_size)

        targetDist = .5 * float(block_size)

        print('Traveling %f blocks.' % num_blocks)

        distance = float(num_blocks) * block_size

        self.reset_encoder(self.LEFT_MOTOR)
        self.reset_encoder(self.RIGHT_MOTOR)
        positionPreviousLeft = 0
        positionPreviousRight = 0

        correction = 0
        distTotal = 0
        previousDist = 0
        condition = 0

        while distTotal < distance:

            left_power = direction * (self.POWER - correction)
            right_power = direction * (self.POWER + correction)

            BP.set_motor_power(self.LEFT_MOTOR, left_power)
            BP.set_motor_power(self.RIGHT_MOTOR, right_power)

            currentDist = self.check_distance()
            positionCurrentLeft = BP.get_motor_encoder(BP.PORT_D)
            positionCurrentRight = BP.get_motor_encoder(BP.PORT_A)

            distDiff = currentDist - previousDist

            leftDistDrive = self.DIST_DEG * abs(positionCurrentLeft - \
                                                 positionPreviousLeft)
            rightDistDrive = self.DIST_DEG * abs(positionCurrentRight - \
                                                  positionPreviousRight)
            distDrive = (leftDistDrive + rightDistDrive) / 2

            angle = atan(distDiff / (distDrive + 0.01))
            distPerp = currentDist * cos(abs(angle))
            distParallel = distDrive * cos(abs(angle))
            distTotal = distTotal + distParallel

            error = distPerp - targetDist

            if currentDist > block_size and angle < 1.57:
                correction = 0
                if condition == 0:
                    distTotal += 0.5 * float(block_size) - (distance - distTotal)
                    condition = 1
            else:
                correction = self.KP * error

            positionPreviousLeft = positionCurrentLeft
            positionPreviousRight = positionCurrentRight
            previousDist = currentDist

        self.stop()

    def turn(self, direction, degrees):

        # This function turns the robot the input number of degrees. It also
        # takes a wheel radius and speed input.

        print('Turning ', degrees, ' degrees.')
        self.reset_encoder(self.LEFT_MOTOR)
        self.reset_encoder(self.RIGHT_MOTOR)
        print('init val:', BP.get_motor_encoder(self.LEFT_MOTOR))
        max_deg = degrees * self.DEG_TURN
        print('max deg:', max_deg)
        deg_traveled = 0

        if direction == 0:
            left_speed = -1 * self.POWER
            right_speed = self.POWER
        else:
            left_speed = self.POWER
            right_speed = -1 * self.POWER
        while deg_traveled < max_deg:
            BP.set_motor_power(self.LEFT_MOTOR, left_speed)
            BP.set_motor_power(self.RIGHT_MOTOR, right_speed)
            deg_traveled = abs(BP.get_motor_encoder(self.LEFT_MOTOR))
            #print('traveled:', deg_traveled)
        self.stop()

    def turn_ultrasonic(self, direction, block_size):

        print('Rotating Ultrasonic.')

        if direction == self.LEFT:
            power = 50
            target = self.ULTRASONIC_LEFT
        elif direction == self.RIGHT:
            power = -50
            target = self.ULTRASONIC_RIGHT

        self.reset_encoder(self.ULTRASONIC_MOTOR)

        diff = 0
        while diff < target:
            BP.set_motor_power(self.ULTRASONIC_MOTOR, power)
            diff = abs(BP.get_motor_encoder(self.ULTRASONIC_MOTOR))
            time.sleep(.01)

        self.stop()

        return

    def check_distance(self):
        dist = grovepi.ultrasonicRead(self.ULTRASONIC)
        print('Distance: ', dist)
        return dist

    def check_junction(self, unit):

        # This function checks the front, left, and right distance on the
        # brickpi ultrasonic sensor and returns the distances

        print('Determining junction type')

        leftDist = self.check_distance()
        self.turn_ultrasonic(self.RIGHT, unit)
        forwardDist = self.check_distance()
        self.turn_ultrasonic(self.RIGHT, unit)
        rightDist = self.check_distance()
        self.turn_ultrasonic(self.LEFT, unit)
        self.turn_ultrasonic(self.LEFT, unit)

        typeJunction = self.JUNCT_DEAD_END

        if forwardDist > unit:
            typeJunction = self.JUNCT_STRAIGHT + typeJunction

        if leftDist > unit:
            typeJunction = self.JUNCT_LEFT + typeJunction
            
        if rightDist > unit:
            typeJunction = self.JUNCT_RIGHT + typeJunction

        return typeJunction

    def turn_junction(self, junction, direction):

        # This function turns the robot to the leftmost fork of a Junction

        typeJunction = junction
        if (typeJunction == self.JUNCT_RIGHT) or \
        (typeJunction == self.JUNCT_LEFT_RIGHT) or \
        (typeJunction == self.JUNCT_RIGHT_STRAIGHT) or \
        (typeJunction == self.JUNCT_ALL_WAY):
            self.turn(self.RIGHT, 90)
            direction = self.change_direction(direction, self.LEFT)
        elif (typeJunction == self.JUNCT_LEFT) or (typeJunction == self.JUNCT_LEFT_STRAIGHT):
            self.turn(self.LEFT, 90)
            direction = self.change_direction(direction, self.RIGHT)
        elif typeJunction == self.JUNCT_DEAD_END:
            self.turn(self.LEFT, 180)
            direction = self.change_direction(direction, self.UTURN)

        return direction, junction

    def explore_space(self, block_size, mapMatrix, direction, positionX,\
                      positionY):
        pastX = [positionX]
        pastY = [positionY]
        mapMatrix[positionY][positionX] = 10
        minimum = -1
        while (positionX != pastX[0] or positionY != pastY[0]) or minimum < 0:
            minimum = 0
            for i in range(len(mapMatrix)):
                current = min(mapMatrix[i])
                if (current < minimum):
                    minimum = current
                    break;

            junction, openSpace = self.check_junction(block_size)
            while junction == self.JUNCT_STRAIGHT:
                self.drive_dist(1, block_size)
                if (positionX != pastX[0]) or (positionY != pastY[0]):
                    mapMatrix[positionY][positionX] = 1
                positionX, positionY = self.change_position(direction, \
                positionX, positionY)
                pastX.append(positionX)
                pastY.append(positionY)
                junction, openSpace = self.check_junction(block_size)

            if (openSpace > 0):
                mapMatrix[positionY][positionX] = openSpace
            junction = self.check_map(junction, mapMatrix, direction, \
            positionX, positionY)
            direction, junction = self.turn_junction(junction, direction)
            if junction == self.JUNCT_DEAD_END:
                self.return_junction(pastX, pastY, mapMatrix, block_size, \
                direction)
            if (mapMatrix[pastY[-1]][pastX[-1]] != 10):
                self.drive_dist(1, block_size)
                numJunction = (int(junction) / 100) + ((int(junction) % 100) / \
                10) + ((int(junction) % 100) % 10)
                if (numJunction == -1):
                    numJunction = 1
                mapMatrix[positionY][positionX] = numJunction
                positionX, positionY = self.change_position(direction, \
                positionX, positionY)
                pastX.append(positionX)
                pastY.append(positionY)

        return direction, positionX, positionY

    def explore_space_simple(self, block_size, mapMatrix, direction, positionX, \
        positionY):
        pastX = [positionX]
        pastY = [positionY]
        mapMatrix[positionY][positionX]
        openSpace = 0
        pastJunction = 0
        while (openSpace == 0):
            junction = self.check_junction(block_size)
            while junction == self.JUNCT_STRAIGHT:
                self.drive_dist(1, block_size)
                if (positionX != pastX[0]) or (positionY != pastY[0]):
                    mapMatrix[positionY][positionX] = 1
                positionX, positionY = self.change_position(direction, \
                positionX, positionY)
                pastX.append(positionX)
                pastY.append(positionY)
                junction = self.check_junction(block_size)
            if (openSpace > 0):
                mapMatrix[positionY][positionX] = openSpace
            direction, junction = self.turn_junction(junction, direction)
            self.drive_dist(1, block_size)
            numJunction = (int(junction) / 100) + (((int(junction) % 100) / 10) / \
                10) + ((int(junction) % 100) % 10)
            if (numJunction == -1):
                numJunction = 1
            mapMatrix[positionY][positionX] = numJunction
            positionX, positionY = self.change_position(direction, \
            positionX, positionY)
            pastX.append(positionX)
            pastY.append(positionY)

            junctionRight = int(abs(junction)) / 100
            junctionLeft = (int(abs(junction)) % 100) / 10
            pastJunctionRight = int(abs(pastJunction)) / 100
            pastJunctionLeft = (int(abs(pastJunction)) % 100) / 10
            if (junctionRight == 1 or junctionLeft == 1):
                right = junctionRight * pastJunctionRight
                left = junctionLeft * pastJunctionLeft
            else:
                right = 0
                left = 0
            openSpace = right + left
            pastJunction = junction

        mapMatrix[positionY][positionX] = 6
        return direction, positionX, positionY, pastX, pastY

    def explore_open(self, block_size, mapMatrix, direction, positionX, positionY, pastX, pastY):
        self.drive_dist(-1,block_size)
        positionX, positionY = self.change_position(direction, \
        positionX, positionY)
        self.turn(direction, 90)
        direction = self.change_direction(direction, self.LEFT)
        self.drive_dist(1, block_size)
        junction = self.check_junction(block_size)
        while (junction != self.JUNCT_ALL_WAY):
            direction, junction = self.turn_junction(junction, direction)
            self.drive_dist(1, block_size)
            mapMatrix[positionY][positionX] = 1
            positionX, positionY = self.change_position(direction, \
            positionX, positionY)
            pastX.append(positionX)
            pastY.append(positionY)
            junction = self.check_junction(block_size)
        return direction, positionX, positionY
        # self.avoid(self, currentX, currentY, mapMatrix)

    def change_position(self, direction, positionX, positionY):
        if (direction == self.LEFT):
            positionX -= 1
        if (direction == self.RIGHT):
            positionX += 1
        if (direction == self.UP):
            positionY += 1
        if (direction == self.DOWN):
            positionY -= 1
        return positionX, positionY

    def check_map(self, junction, mapMatrix, direction, positionX, positionY):
        if (direction == self.LEFT):
            forwardMap = mapMatrix[positionX - 1][positionY]
            leftMap = mapMatrix[positionX][positionY - 1]
            rightMap = mapMatrix[positionX][positionY + 1]
        if (direction == self.UP):
            forwardMap = mapMatrix[positionX][positionY + 1]
            leftMap = mapMatrix[positionX - 1][positionY]
            rightMap = mapMatrix[positionX + 1][positionY]
        if (direction == self.RIGHT):
            forwardMap = mapMatrix[positionX + 1][positionY]
            leftMap = mapMatrix[positionX][positionY + 1]
            rightMap = mapMatrix[positionX][positionY - 1]
        if (direction == self.DOWN):
            forwardMap = mapMatrix[positionX][positionY - 1]
            leftMap = mapMatrix[positionX + 1][positionY]
            rightMap = mapMatrix[positionX - 1][positionY]

        forwardMap = -1 * (forwardMap - 1)
        leftMap = -1 * (leftMap - 1)
        rightMap = -1 * (rightMap - 1)

        forwardJunct = (int(abs(junction)) % 100) % 10
        leftJunct = (int(abs(junction)) % 100) / 10
        rightJunct = int(abs(junction)) / 100

        straight = int(forwardMap * forwardJunct)
        left = int(leftMap * leftJunct)
        right = int(rightMap * rightJunct)

        junction = -1 * (straight + (10 * left) + (100 * right))

        return junction

    def change_direction(self, direction, turnType):
        if (direction == self.LEFT):
            if (turnType == self.LEFT):
                direction += 3
            elif (turnType == self.RIGHT):
                direction += 2
            elif (turnType == self.UTURN):
                direction += 1
        elif (direction == self.RIGHT):
            if (turnType == self.LEFT):
                direction += 1
            elif (turnType == self.RIGHT):
                direction += 2
            elif (turnType == self.UTURN):
                direction -= 1
        elif (direction == self.UP):
            if (turnType == self.LEFT):
                direction -= 2
            elif (turnType == self.RIGHT):
                direction -= 1
            elif (turnType == self.UTURN):
                direction += 1
        elif (direction == self.DOWN):
            if (turnType == self.LEFT):
                direction -= 2
            elif (turnType == self.RIGHT):
                direction -= 3
            elif (turnType == self.UTURN):
                direction -= 1

        return direction

    def return_junction(self, pastX, pastY, mapMatrix, block_size, direction):

        currentX = pastX[-1]
        currentY = pastY[-1]
        newX = pastX[-2]
        newY = pastY[-2]
        position = mapMatrix[currentX][currentY]

        while(position == 1):
            direction = self.drive_coord(block_size, direction, [currentX, \
            currentY], [newX, newY])

            currentX = newX
            currentY = newY
            position = mapMatrix[currentX, currentY]
            del pastX[-1]
            del pastY[-1]
            newX = pastX[-2]
            newY = pastY[-2]

        if (position != 10):
            junction, openSpace = self.check_junction(block_size)
            junction, openSpace = self.check_map(junction, mapMatrix, direction, \
                                      currentX, currentY)
            direction, junction = self.turn_junction(junction)

        return direction, junction

    def drive_coord(self, block_size, direction, initCoord, finalCoord):

        # Drive from initial coord to final coord

        initX = initCoord[0]
        initY = initCoord[1]
        finalX = finalCoord[0]
        finalY = finalCoord[1]

        xTravel = finalX - initX
        yTravel = finalY - initY

        if (xTravel < 0):
            if (direction == self.LEFT):
                self.drive_dist(-xTravel, block_size)
            elif (direction == self.UP):
                self.turn(self.LEFT, 90)
                direction -= 2
                self.drive_dist(-xTravel, block_size)
            elif (direction == self.DOWN):
                self.turn(self.RIGHT, 90)
                direction -= 3
                self.drive_dist(-xTravel, block_size)
            elif (direction == self.RIGHT):
                self.turn(self.LEFT, 180)
                direction -= 1
                self.drive_dist(-xTravel, block_size)
        if (xTravel > 0):
            if (direction == self.RIGHT):
                self.drive_dist(xTravel, block_size)
            elif (direction == self.UP):
                self.turn(self.RIGHT, 90)
                direction -= 1
                self.drive_dist(xTravel, block_size)
            elif (direction == self.DOWN):
                self.turn(self.LEFT, 90)
                direction -= 2
                self.drive_dist(xTravel, block_size)
            elif (direction == self.LEFT):
                self.turn(self.RIGHT, 180)
                direction += 0
                self.drive_dist(xTravel, block_size)
        if (yTravel < 0):
            if (direction == self.DOWN):
                self.drive_dist(-yTravel, block_size)
            elif (direction == self.LEFT):
                self.turn(self.LEFT, 90)
                direction += 3
                self.drive_dist(-yTravel, block_size)
            elif (direction == self.RIGHT):
                self.turn(self.RIGHT, 90)
                direction += 2
                self.drive_dist(-yTravel, block_size)
            elif (direction == self.UP):
                self.turn(self.LEFT, 180)
                direction += 1
                self.drive_dist(-yTravel, block_size)
        if (yTravel > 0):
            if (direction == self.UP):
                self.drive_dist(yTravel, block_size)
            elif (direction == self.LEFT):
                self.turn(self.RIGHT, 90)
                direction += 2
                self.drive_dist(yTravel, block_size)
            elif (direction == self.RIGHT):
                self.turn(self.LEFT, 90)
                direction += 1
                self.drive_dist(yTravel, block_size)
            elif (direction == self.DOWN):
                self.turn(self.RIGHT, 180)
                direction -= 1
                self.drive_dist(yTravel, block_size)

        return direction

    def csv_write(self, fid, array):

        for row in array:
            line = str(row)
            line = line.strip('[]')
            fid.write(line)
            fid.write('\n')

    def map_output(self, map_number, unit_length, unit, origin, notes, \
                   mapMatrix):

        # This function outputs a map of resources and walls

        fileName = 'mapOutput.csv'

        mapOutput = open(fileName, 'w')
        mapOutput.write('Team: %d\n' % self.TEAM)
        mapOutput.write('Map: %d\n' % map_number)
        mapOutput.write('Unit Length: %d\n' % unit_length)
        mapOutput.write('Unit: %s\n' % unit)
        mapOutput.write('Origin: (%.f, %.f)\n' % (origin[1], origin[0]))
        mapOutput.write('Notes: %s\n' % notes)
        self.csv_write(mapOutput, mapMatrix)
        mapOutput.close()

    def resource_output(self, resources, map_number, notes):

        # This function outputs a file of resources

        fileName = 'resources.csv'
        resourceOutput = open(fileName, 'w')
        resourceOutput.write('Team %d\n' % self.TEAM)
        resourceOutput.write('Map %d\n' % map_number)
        resourceOutput.write('Notes: %s\n\n' % notes)
        self.csv_write(resourceOutput, resources)
        resourceOutput.close()

    def set_motor(self, motor, deg):
        deg_diff = 6
        while deg_diff > 5:
            deg_diff = abs(BP.get_motor_encoder(self.ARM_MOTOR) - deg)
            BP.set_motor_position(self.ARM_MOTOR, deg)

    def fix_arm(self):
        self.reset_encoder(self.ARM_MOTOR)
        self.set_motor(self.ARM_MOTOR, 130)

    def mass(self, time, power):

        # time in s
        grav = 9.8 # m/s^2
        height = 5.5 # cm
        angle = 45 / 360 * 2 * pi # radians
        torque = ((angle / time) - 165) / (-.4539 * .01 * power) # N * m

        mass = torque * angle / (height * grav) * 1000 # grams

        return mass

    def weigh(self):

        # This function detects
        power = 10
        mass = 'error massing'

        scan = self.scanner()
        if scan != self.WALL_COLOR:
            self.drive_dist(1, -10) # cm
            self.turn(self.LEFT, 180)
            self.stop()
            print('turn complete')
            # lower the motor
            self.set_motor(self.ARM_MOTOR, 0)

            # drive towards resource
            self.drive_dist(1, -15) # cm
            self.stop()
            print('driving to it')
            initial_time = time.time()
            # lift resource and keep track of time elapsed
            initial_time = time.time()
            BP.set_motor_power(self.ARM_MOTOR, power)
            while BP.get_motor_encoder(self.ARM_MOTOR) != 45:
                timer = time.time() - initial_time
            self.stop()

            mass = self.mass(timer, power)

            print('time: ', timer)

            # set the resource back down
            self.set_motor(self.ARM_MOTOR, 0)
            self.stop()

            # drive away from resource
            self.drive_dist(1, 15) # cm

            # lift hook back to initial position
            self.set_motor(self.ARM_MOTOR, 130)
            self.stop()

            self.turn(0,180)

        return mass

    def avoid(self, currentX, currentY, mapMatrix):

        # Avoids the cesium and the MRI

        mri_detected = self.magnet()
        cesium_detected = self.ir_read()

        if mri_detected or cesium_detected:
            self.stop()
        if cesium_detected:
            hazard = self.RADIATION
        else:
            hazard = self.MRI

        mapMatrix[currentY, currentX] = hazard


    def kill(self):

        # This function resets all motors and sensors. It should only be used
        # at the end of the code, or as the result of a keyboard interrupt.
        # THIS FUNCTION WILL BREAK CODE IF YOU USE IT IN THE MIDDLE.

        print('Killing robot.')

        BP.reset_all()

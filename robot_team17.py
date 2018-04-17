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
# Description

# Imports
import brickpi3
import grovepi
from MPU9250 import MPU9250
from math import pi
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
    BUTTON = BP.PORT_1
    LIGHT = BP.PORT_2
    WHEEL_RADIUS = 1.8
    TRACK_SEPARATION = 18
    ULTRASONIC = 2
    ULTRASONIC_RIGHT = 125

    # Other Constants
    LEFT = 0
    RIGHT = 1
    STRAIGHT = 2
    BIOHAZARD_COLOR = 'Biohazard' # Yellow
    NONHAZARD_COLOR = 'Nonhazardous' # Blues
    HAZARD_THRESHOLD = 2700
    CESIUM_THRESHOLD = 180
    MRI_THRESHOLD = 65
    DIST_DEG = (2 * pi * WHEEL_RADIUS) / 360
    wheel_track_ratio = TRACK_SEPARATION / WHEEL_RADIUS
    FULL_TURN = 2055
    DEG_TURN = FULL_TURN / 360

    # Junction types
    JUNCT_DEAD_END = 0
    JUNCT_STRAIGHT = 1
    JUNCT_LEFT = 10
    JUNCT_RIGHT = 100
    JUNCT_LEFT_RIGHT = 110
    JUNCT_LEFT_STRAIGHT = 11
    JUNCT_RIGHT_STRAIGHT = 101
    JUNCT_ALL_WAY = 111


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
        unit_length = self.getval('Input unit length: ')
        unit = input('Input unit: ')
        origin.append(self.getval('Input x origin coordinate: '))
        origin.append(self.getval('Input y origin coordinate: '))
        notes = input('Notes: ')

        return map_number, unit_length, unit, origin, notes

    def mapSetup(xSize, ySize):

        # This function sets up the map matrix

        mapMatrix = np.zeros(xSize, ySize)
        return mapMatrix

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
        if scan < self.HAZARD_THRESHOLD:
            return self.BIOHAZARD_COLOR
        else:
            return self.NONHAZARD_COLOR

    def ir_read(self):

        # This function reads the IR sensor data

        [sens1, sens2] = ir.IR_Read(grovepi)
        if sens2 > 180:
            print('Danger')
        return sens2

    def magnet(self):

        # This function returns the magnetic sensor value

        raw = imu.readMagnet()

        magnet = raw['y']

        print(magnet)

    def drive(self, speed):
        BP.set_motor_dps(self.LEFT_MOTOR + self.RIGHT_MOTOR, speed)

    def stop(self):
        BP.set_motor_dps(self.LEFT_MOTOR + self.RIGHT_MOTOR + \
                         self.ULTRASONIC_MOTOR, 0)

    def dist_deg_calculator(self, distance):
        init_deg = BP.get_motor_encoder(self.LEFT_MOTOR)
        max_deg = distance / self.DIST_DEG
        deg_traveled = 0

        return init_deg, max_deg, deg_traveled

    def drive_dist(self, distance, speed):

        # This function drives the robot at the input speed for the input
        # distance

        print('Traveling distance ', distance)

        init_deg, max_deg, deg_traveled = self.dist_deg_calculator(distance)

        while deg_traveled < max_deg:
            self.drive(speed)
            deg_traveled = abs(BP.get_motor_encoder(self.LEFT_MOTOR) - \
                               init_deg)
        self.stop()

    def turn(self, direction, degrees, speed):

        # This function turns the robot the input number of degrees. It also
        # takes a wheel radius and speed input.

        print('Turning ', degrees, ' degrees.')
        init_deg = BP.get_motor_encoder(self.LEFT_MOTOR)
        BP.offset_motor_encoder(self.LEFT_MOTOR + self.RIGHT_MOTOR, init_deg)
        print('init val:', BP.get_motor_encoder(self.LEFT_MOTOR))
        max_deg = degrees * self.DEG_TURN
        print('max deg:', max_deg)
        deg_traveled = 0

        if direction == 0:
            left_speed = -speed
            right_speed = speed
        else:
            left_speed = speed
            right_speed = -speed
        while deg_traveled < max_deg:
            BP.set_motor_dps(self.LEFT_MOTOR, left_speed)
            BP.set_motor_dps(self.RIGHT_MOTOR, right_speed)
            deg_traveled = abs(BP.get_motor_encoder(self.LEFT_MOTOR))
            #print('traveled:', deg_traveled)
        self.stop()

    def turn_ultrasonic(self, direction):

        print('Rotating Ultrasonic.')

        if direction == self.LEFT:
            power = 50
        elif direction == self.RIGHT:
            power = -50

        init = BP.get_motor_encoder(self.ULTRASONIC_MOTOR)
        diff = 0

        while diff < self.ULTRASONIC_RIGHT:
            BP.set_motor_power(self.ULTRASONIC_MOTOR, power)
            diff = abs(BP.get_motor_encoder(self.ULTRASONIC_MOTOR) - init)
            time.sleep(.01)

        self.stop()

    def check_distance(self):
        dist = grovepi.ultrasonicRead(self.ULTRASONIC)
        print('Distance: ', dist)
        return dist

    def check_junction(self, unit):

        # This function checks the front, left, and right distance on the
        # brickpi ultrasonic sensor and returns the distances

        print('Determining junction type')

        forwardDist = self.check_distance()
        self.turn_ultrasonic(self.LEFT)
        leftDist = self.check_distance()
        self.turn_ultrasonic(self.RIGHT)
        self.turn_ultrasonic(self.RIGHT)
        rightDist = self.check_distance()
        self.turn_ultrasonic(self.LEFT)

        numJunction = 0
        typeJunction = self.JUNCT_DEAD_END

        if forwardDist > unit:
            numJunction = numJunction + 1
            typeJunction = self.JUNCT_STRAIGHT + typeJunction
        if leftDist > unit:
            numJunction = numJunction + 1
            typeJunction = self.JUNCT_LEFT + typeJunction
        if rightDist > unit:
            numJunction = numJunction + 1
            typeJunction = self.JUNCT_RIGHT + typeJunction

        return numJunction, typeJunction

    def map_output(self, map_number, unit_length, unit, origin, notes):

        # This function outputs a map of resources and walls

        print('Team: ', self.TEAM)
        print('Map: ', map_number)
        print('Unit Length: ', unit_length)
        print('Unit: ', unit)
        print('Origin: (%.f, %.f)' % (origin[0], origin[1]))
        print('Notes: ', notes)

    def kill(self):

        # This function resets all motors and sensors. It should only be used
        # at the end of the code, or as the result of a keyboard interrupt.
        # THIS FUNCTION WILL BREAK CODE IF YOU USE IT IN THE MIDDLE.

        print('Killing robot.')

        BP.reset_all()

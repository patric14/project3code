# -*- coding: utf-8 -*-
"""
Created on Sat Apr 21 14:58:20 2018

@author: haydn_000
"""

import robot_team17
import time

robot = robot_team17.RobotLibrary()

initial_time = time.time()

robot.scanner()
if scan > WALL_THRESHOLD:
    robot.drive_dist(1,-10) # cm
    robot.turn(0,180)
    
    # lower the motor
    robot.reset_motor_encoder(self.ARM_MOTOR,0)
    final_deg = 108
    deg = BP.get_motor_encoder(BP.PORT_C)
    while deg < 108:
        BP.set_motor_power(self.ARM_MOTOR, -50)
    
    # drive towards resource
    robot.drive_dist(1,15) # cm
    
    # lift resource and keep track of time elapsed
    while deg > 23:
        BP.set_motor_power(self.ARM_MOTOR,50)
        elapsed_time = time.time() - initial_time
    
    # set the resource back down
    while deg < 108:
        BP.set_motor_power(self.ARM_MOTOR,-50)
    
    # drive away from resource
    drive_dist(1,15) # cm
    
    # lift hook back to initial position
    while deg > 0:
        BP.set_motor_power(self.ARM_MOTOR,50)
    
    robot.turn(0,180)
    
    
    
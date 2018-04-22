#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 18:02:23 2018

@author: caleb
"""

import brickpi3

BP = brickpi3.BrickPi3()

try:
    while True:
        BP.set_motor_power(BP.PORT_A+BP.PORT_D, 100)
except KeyboardInterrupt:
    pass

BP.reset_all()
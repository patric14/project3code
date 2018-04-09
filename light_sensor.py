import time # allow use of functions in time module
import brickpi3 # allow use of routines to control NXT motors and read sensors
import grovepi # allow use of routines to read grovepi sensors
BP = brickpi3.BrickPi3()

BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.NXT_LIGHT_ON)


try:
    while True:
        try:
        lightSens = BP.get_sensor(BP.PORT_3)


        print('Value: ', lightSens)
        time.sleep(.5)

except KeyboardInterrupt:
    pass

BP.reset_all()
import robot_team17
import time
robot = robot_team17.RobotLibrary()

try:

    while True:

        robot.drive_dist(.5, 15, 40)
        typeJunction = robot.check_junction(40)

        if typeJunction == 0:
            junct = 'Dead end'
        elif typeJunction == 1:
            junct = 'Straight'
        elif typeJunction == 10:
            junct = 'Left'
            robot.turn(robot.LEFT, 90)
        elif typeJunction == 100:
            junct = 'Right'
        elif typeJunction == 110:
            junct = 'Left and Right'
        elif typeJunction == 11:
            junct = 'Left and Straight'
        elif typeJunction == 101:
            junct = 'Left and Straight'
        elif typeJunction == 111:
            junct = 'All way'
        else:
            junct = 'Ya done goofed'

        print(junct)

except KeyboardInterrupt:
    pass

robot.kill()

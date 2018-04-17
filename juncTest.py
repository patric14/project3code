import robot_team17
robot = robot_team17.RobotLibrary()

try:

    numJunction, typeJunction = robot.check_junction(35)
    
    if typeJunction == 0:
        junct = 'Dead end'
    elif typeJunction == 1:
        juct = 'Straight'
    elif typeJunction == 10:
        junct = 'Left'
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
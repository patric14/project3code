import robot_team17
robot = robot_team17.RobotLibrary()

# deg = robot.getval('Input degrees: ')
direction = robot.getval('Direction, 0 for left, 1 for right: ')

try:

    robot.turn_ultrasonic(direction)

except KeyboardInterrupt:
    pass

robot.kill()

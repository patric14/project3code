import robot_team17
import time

robot = robot_team17.RobotLibrary()

speed = robot.getval('Speed: ')

try:
    robot.drive_dist(70, speed)
    robot.stop()
    time.sleep(.1)
    robot.turn(robot.RIGHT, 90, speed)
    robot.drive_dist(110, speed)
    robot.stop()
    time.sleep(.1)
    robot.turn(robot.LEFT, 90, speed)
    robot.drive_dist(35, speed)
    robot.stop()
    time.sleep(3)

except KeyboardInterrupt:
    pass

robot.kill()

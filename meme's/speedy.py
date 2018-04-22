import robot_team17 as robot

try:
    speed = float(input('Speed: '))

    while True:
        robot.drive(speed)

except KeyboardInterrupt:
    pass

robot.kill()
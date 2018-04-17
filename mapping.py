import robot_team17
import time
IN2FT = 12
mapLengthX = 22 * IN2FT #in
mapLengthY = 12 * IN2FT #in

map_number, unit_length, unit, origin, notes = setup()

mapBlockX = mapLengthX / unit_length
mapBlockY = mapLengthY / unit_length

map = mapSetup(mapBlockX, mapBlockY)



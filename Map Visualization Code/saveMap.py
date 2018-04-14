# Sample file that saves a map as the students would.

import time
import csv

map = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
       [0, 0, 0, 0, 1, 5, 0, 0, 0, 0],
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
       [0, 0, 0, 0, 1, 1, 1, 2, 0, 0],
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]





fID = open("map.csv", 'w')
writer = csv.writer(fID)

fID.write("Team: 99\n")
fID.write("Map: 0\n")
fID.write("Unit Length: 40\n")
fID.write("Unit: cm\n")
fID.write("Origin: xx\n")
fID.write("Notes: Example Map\n")

for row in map:
    writer.writerow(row)

fID.close()

# Sends the map to the computer using the CSV file on the Pi
# Run with recieveCSV.py on the computer.
# 
# Update the IPv4 address in line 10 as needed.

from piTalk import connect, sendMapFile
import time
import csv

ipv4_address = '192.168.XX.YY'
refreshRate = 1 # second(s)
showRawSendData = False

connect( ipv4_address)
startTime = time.time()

while True:
  if (int(time.time() - startTime) >= refreshRate):
    map = []
    count = 0

    mapFile = open('/home/mk_99/Desktop/map.csv', 'r')
    reader = csv.reader(mapFile)
    for row in reader:
      if (count < 6):
        map.append(row)
      else:
        try:
          map.append([int(i) for i in row])
        except:
          tempRow = []
          for c in row:
            try:
              tempRow.append(int(c))
            except:
              tempRow.append(c)
          map.append(tempRow)
      count += 1
  
    mapFile.close()
    sendMapFile(map, showData=showRawSendData)
    startTime = time.time()

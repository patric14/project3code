# This file connects to the Pi, recieves a map, and saves it as a csv.
# Run with sendCSV.py on the Pi
# 
# Update the IPv4 address in line 10 as needed.

import sys
import time
from baseTalk import connect, waitForData

ipv4_address = '192.168.XX.YY'
showRawRecieveData = False

connect(ipv4_address)
import subprocess
p = subprocess.Popen('py liveMapCSV.py')


while True:
    content = waitForData(showData=showRawRecieveData)
    
    if (content == 0):
        p.terminate()
        sys.exit("\nConnection was closed or invalid packet was received.")
    
    if not(type(content) is list) and not(type(content[0]) is list):
        print("Invalid Map received")
        print("\nRecieved: \n", content)
        print("Invalid Map received")
        
    else:
        header = content[:6]
        map = content[6:]
        
        try:
            file = open("map.csv", 'w')
        except:
            time.sleep(0.5)
            file = open("map.csv", 'w')
        
        for h in header:
            file.write(str(h[0] + '\n'))
        for row in map:
            for c in range(len(row)-1):
                file.write(str(row[c]) + ',')
            file.write(str(str(row[-1]) + '\n'))
        
        file.close()

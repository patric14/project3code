# This file reads the track array (below) and a csv file named "map.csv" and plots them in real time.
# 
# For use when pi is sending from a CSV File with the FULL header
# Use with recieveCSV.py

import os
import sys
import csv
import shutil
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# If running in Spyder, enter   %matplotlib auto   into the IPython console

folderName = 'Team_99'
refreshRate = 1 # sec

showRawPlotData = True

track  = [[ 0, 0, 1, 2, 2, 2, 1, 0, 0, 0], 
          [ 0, 0, 1, 2, 2, 2, 1, 0, 0, 0], 
          [ 0, 0, 1, 1, 1, 1, 1, 0, 0, 0], 
          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
          [ 0, 0, 0, 0, 1, 1, 1, 1, 0, 0], 
          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0], 
          [ 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]]
          
    
def getTrackPlot( track):      
    paddedTrack = np.array(track)
    paddedTrack = np.pad(paddedTrack, 1, 'constant', constant_values=0)
    
    vertX = []
    vertY = []
    horX = []
    horY = []
    crossX = []
    crossY = []
    x = []
    y = []       

    # Starts at 1 because of the buffer
    for i in range(1, len(paddedTrack)-1):
      for j in range(1, len(paddedTrack[0])-1):
        thisx = j - 1
        thisy = len(paddedTrack) - i - 2
        tempx = j - 1
        tempy = len(paddedTrack) - i - 2
        if (paddedTrack[i][j] == 0):
            LR = False
            TB = False
            if ((paddedTrack[i-1][j] == 1)):
                tempy += 0.5
                TB = True
                horX.append(thisx)
                horY.append(tempy)
            if (paddedTrack[i][j-1] == 1):
                tempx -= 0.5
                LR = True
                vertX.append(tempx)
                vertY.append(thisy)
            if (paddedTrack[i][j+1] == 1):
                tempx += 0.5
                LR = True
                vertX.append(tempx)
                vertY.append(thisy)
            if ((paddedTrack[i+1][j] == 1)):
                tempy -= 0.5
                TB = True
                horX.append(thisx)
                horY.append(tempy)
            
            if (TB == True) and (LR == True):
                crossX.append(tempx)
                crossY.append(tempy)
                    
    trackVars = [vertX, vertY, horX, horY, crossX, crossY]
    return trackVars
  
    
def getStudentPlot( map, xSize, ySize):
    tempx = -1
    tempy = -1
    xScale = xSize / (len(map[0]))
    yScale = ySize / (len(map   ))
    
    path = [[], []]
    stuff = []
    error = []
            # X, Y, label
    
    for r in range(len(map)-1, -1, -1):
      tempy += 1
      tempx = -1
      for c in range(len(map[r])):
        tempx += 1
        try:
            value = int(map[r][c])
        except:
            error.append([tempx, tempy, map[r][c]])
            
        if (value == 1):
            path[0].append(tempx)
            path[1].append(tempy)
        elif (value == 10):
            stuff.append([tempx, tempy, 'Origin'])
        elif (value == 2):
            stuff.append([tempx, tempy, 'Biohazard'])
        elif (value == 3):
            stuff.append([tempx, tempy, 'Non-Haz Waste'])
        elif (value == 4):
            stuff.append([tempx, tempy, 'Radiation Source'])
        elif (value == 5):
            stuff.append([tempx, tempy, 'MRI'])
        elif (value == 6):
            stuff.append([tempx, tempy, 'Open Space Start'])
        elif (value == 7):
            stuff.append([tempx, tempy, 'Exit Point'])
        elif (value != 0):
            stuff.append([tempx, tempy, map[r][c]])
            
    path[0]  = [x * xScale for x in path[0]]
    path[1]  = [y * yScale for y in path[1]]
    for r in stuff:
        r[0] = r[0] * xScale
        r[1] = r[1] * yScale
    for r in error:
        r[0] = r[0] * xScale
        r[1] = r[1] * yScale
    
    return path, stuff, error


def animate(i):
    global track, openErr
    plotTrack = getTrackPlot( track)
    mapOpen = False
    
    try:
        file = open("map.csv", 'r')
        mapOpen = True
        if openErr:
            print("\nFile Opened Successfully.")
            openErr = False
    except:
        if openErr:
            print('.', end='')
            sys.stdout.flush()
        else:
            print("Error opening map file.", end='')
            openErr = True
        mapOpen = False
        
    ax.clear()
    
    if (mapOpen):
        rawCSV = []
        
        reader = csv.reader(file)
        for row in reader:
            rawCSV.append(row)
        
        file.close()
        
            
        header = rawCSV[:6]
        global folderName
        folderName = str("Team_" + header[0][0][-2:])
        
        studentMap = rawCSV[6:]
        
        if (showRawPlotData):
            for r in studentMap:
                print(r)
            print("Track: ", len(track[0]), " x ", len(track))
            print("Map: ", len(studentMap[0]), " x ", len(studentMap))
        path, stuff, error = getStudentPlot( studentMap, len(track[0]), len(track))
        
        ax.plot(path[0], path[1], 'b.')
        for e in stuff:
            ax.plot(e[0], e[1], 'g*')
            ax.annotate(e[2], xy=(e[0]-0.1, e[1]-0.3))
        for e in error:
            ax.plot(e[0], e[1], 'r*')
            ax.annotate(e[2], xy=(e[0]-0.1, e[1]-0.3))
        
    ax.plot(plotTrack[0], plotTrack[1], 'r|')
    ax.plot(plotTrack[2], plotTrack[3], 'r_')
    ax.plot(plotTrack[4], plotTrack[5], 'r+')
    
    ax.set_xlim([-0.25, len(track[0])-1+0.25])
    ax.set_ylim([-0.25, len(track   )-1+0.25])
    plt.savefig('map.pdf')

    
def handle_close(evt):
    if not(os.path.isdir(folderName)):
        os.mkdir(folderName)
    
    run = 1
    while(os.path.isfile(str(folderName + '/map_' + str(run) + '.csv'))):
        run += 1
        
    shutil.move("map.csv", str(folderName + '/map_' + str(run) + '.csv'))
    shutil.move("map.pdf", str(folderName + '/map_' + str(run) + '.pdf'))
    print('Files Saved.')

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim([-0.25, len(track[0])-1+0.25])
ax.set_ylim([-0.25, len(track   )-1+0.25])
studentMap = -1
openErr = False
fig.canvas.mpl_connect('close_event', handle_close)
ani = FuncAnimation(fig, animate, interval=refreshRate*1000)
plt.show()
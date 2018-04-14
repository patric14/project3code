# Library file on the Computer.
# Must be in the same directory as any file using it's functions.

import socket
import struct
import sys
from numpy import array

buffer = 1024

def flatten( array):
  flat = []
  if (type(array[0]) is list):
    if (type(array[0][0]) is list):
      if (type(array[0][0][0]) is list):
        if (type(array[0][0][0][0]) is list):
          sys.exit("Unable to handle arrays of 5 dimensions or greater.\nContact Trevor Meyer <meyer221@purdue.edu>.")
        else:
          for x in array:
            for l in x:
              for r in l:
                for c in r:
                  flat.append(c[0])
      else:
        for l in array:
          for r in l:
            for c in r:
              flat.append(c[0])
    else:
      for r in array:
        for c in r:
          flat.append(c[0])
  else:
    flat = array
  return flat
  
  
def convert2list( array):
    final = []
    dimLength = array[0]
    dim = []
    i = 1
    while (i <= dimLength):
        dim.append(array[i])
        i += 1
        
    if (dimLength == 1):
        while (i < len(array)):
            final.append(array[i])
            i += 1
    elif (dimLength == 2):
        t1 = []
        count = 0
        new = dim[-1]
        while (i < len(array)):
            t1.append(array[i])
            count += 1
            i += 1
            if (count >= new):
                final.append(t1)
                t1 = []
                count = 0
    elif (dimLength == 3):
        t1 = []
        t2 = []
        count1 = 0
        count2 = 0
        new1 = dim[-1]
        new2 = dim[-2]
        while (i < len(array)):
            t1.append(array[i])
            
            count1 += 1
            i += 1
            if (count1 >= new1):
                t2.append(t1)
                count2 += 2
                t1 = []
                count1 = 0
                
            if (count2 >= new2):
                final.append(t2)
                t2 = []
                count2 = 0
                
    elif (dimLength == 4):
        t1 = []
        t2 = []
        t3 = []
        count1 = 0
        count2 = 0
        count3 = 0
        new1 = dim[-1]
        new2 = dim[-2]
        new3 = dim[-3]
        while (i < len(array)):
            t1.append(array[i])
            
            count1 += 1
            i += 1
            if (count1 >= new1):
                t2.append(t1)
                count2 += 2
                t1 = []
                count1 = 0
                
            if (count2 >= new2):
                t3.append(t2)
                count3 += 1
                t2 = []
                count2 = 0
                
            if (count3 >= new3):
                final.append(t3)
                t3 = []
                count3 = 0
                
    return final


def processFmt( conn, data, b=buffer):
    fmtString = ""
    numPackets = data[0] + data[1] + data[2] + data[3]
    
    first = True
    while(numPackets > 0):
        if (not(first)):
            data = conn.recv(b)
            conn.sendall(b"Received.")
            
        for i in range(first * 4, len(data)):
            fmtString = str(fmtString + chr(data[i]))
        first = False
        numPackets -= 1
    
    return fmtString


def split2packets( fmtString, b=buffer):
    dataSize = { 'i':4, 'f':4, 's':1, '?':1 }
    
    recieve = []
    start = 0
    startFmtStr = ''
    length = 0
    i = 0
    size = 0
    
    
    while (i < len(fmtString)):
        if (fmtString[i].isdigit()):
            length = 10 * length + int( fmtString[i])
        else:
            if (length == 0):
                length = 1
                
            size += length * dataSize[ fmtString[i] ]
            
            if (size > b):
                if (length > 1):
                    firstPacket  = int( (length * dataSize[fmtString[i]] - (size - b)) / dataSize[fmtString[i]] )
                    
                    recieve.append( str(startFmtStr + fmtString[start:fmtEnd] + str(firstPacket) + fmtString[i]))
                    # size = (length - firstPacket) * dataSize[fmtString[i]]
                    fmtString = str( str(length - firstPacket) + fmtString[i] + fmtString[i+1:])
                    size = 0
                    start = 0
                    i = -1
                
                else:
                    recieve.append(str( startFmtStr + fmtString[start:i]))
                    size = dataSize[fmtString[i]]
                    startFmtStr = ''
                    start = i
                
                size = 0
                
            length = 0
            fmtEnd = i+1
        i += 1
    
    recieve.append(str(startFmtStr + fmtString[start:]))
    return recieve

    
def processData( conn, recieve, data, b=buffer):
    dataSize = { 'i':4, 'f':4, 's':1, '?':1 }
    
#    numPackets = data[0] + data[1] + data[2] + data[3]
#    if (not( numPackets == len(recieve))):
#        print("ERROR: Incorrect Packet Math! Contact Trevor Meyer <meyer221@purdue.edu>")
    numPackets = len(recieve)
    
    first = True
    content = []
    r = 0
    d = 0
    
    while(numPackets > 0):
        if (not(first)):
            data = conn.recv(b)
            conn.sendall(b"Received.")
        
        length = 0
        firstElement = True
        firstDigit = True
        isList = False
        isString = False
        i = 0
        d = 0
        
        while (i < len(recieve[r])):
            if (recieve[r][i].isdigit()):
                if (firstDigit and (int(recieve[r][i]) == 0)):
                    length = -1
                length = 10 * length + int(recieve[r][i])
                isList = True
                firstDigit = False
                
            else:
                if (length == 0):
                    length = 1
                elif (length < 0):
                    length = 0
                k = 0
                while (k < length):
                    if (recieve[r][i] == 's'):
                        isString = True
                        string = ""
                        for x in range(length):
                            string = str(string + chr(data[d]))
                            d += 1
                            k += 1
                        if (isList and firstElement and (recieve[r-1][-1] == 's')):
                            content[-1] = str(content[-1] + string)
                        else:
                            content.append(string)
                        
                    else:
                        content.append( struct.unpack(recieve[r][i], data[d:(d+dataSize[recieve[r][i]])]))
                        d += dataSize[recieve[r][i]]
                        k += 1
            
                length = 0
                firstDigit = True
                firstElement = False
                
            i += 1
        
        first = False
        numPackets -= 1
        r += 1

    values = []
    if (isList):
        values = []
        for t in content:
            if (type(t[0]) is str):
                values.append(t[0:])
            else:
                values.append(t[0])
            
        values = convert2list(values)
        
    elif (not(isList) and isString):
        values = ""
        for t in content:
            values = str(values + t)
        
    else:
        values = content[0][0]
        
    return values


def waitForData(bufferSize=buffer, showData=False):
    
    data = conn.recv(bufferSize)
    conn.sendall(b"Received.")
    
    if not data: return 0
    
    formatString = processFmt( conn, data, b=bufferSize)
    recieve = split2packets(formatString, b=bufferSize)
    
    if (showData):
        print("\nBuffer Size: ", bufferSize, "\nFormat:")
        [print(f) for f in recieve]
    
    while True:
        data = conn.recv(bufferSize)
        conn.sendall(b"Received.")
        if data: break
    
    content = processData( conn, recieve, data, b=bufferSize)
        
    if (showData):
        print("Recieved:")
        try:
            [print(str(c)) for c in content]
        except:
            print(content)
    
    return content
    
    
def connect( host):
    global conn
    
    port = 12345  # Arbitrary, will be reassigned by the connection.
    print('Attempting to connect using ', host)
    try:
        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        soc.bind((host, port))
    except:
        sys.exit('Client IP Address was not valid. Check that the correct IP address was entered')
    
    try:
        print('Waiting for connection from host')
        soc.listen(1)
        conn, addr = soc.accept()
    except:
        print('Conneciton request timed out.')
    
    print('Connected by ', addr[0])
    print('Press [ctrl + C] on Pi to stop\n')
# Library file on the Pi.
# Must be in the same directory as any file using it's functions.

import socket 
import struct
import time
import sys
from numpy import array, zeros

buffer = 1024

def _getFormat( x):
  if (type(x) is int):
    return 'i'
  elif (type(x) is float):
    return 'f'
  elif (type(x) is str):
    return 's'
  elif (type(x) is bool):
    return '?'
  else:
    return 's'


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
                  flat.append(c)
      else:
        for l in array:
          for r in l:
            for c in r:
              flat.append(c)
    else:
      for r in array:
        for c in r:
          flat.append(c)
  else:
    flat = array
  return flat


def sendFmtStr( data, b=buffer):
  npy = data
  fmtString = ""
  
  if ( type(data) is list):
    dim = array(npy).shape
    fmtString += 'i'
    for i in dim:
      fmtString += 'i'
      
    d = flatten(data)
    charType  = _getFormat(d[0])
    count = 1
    first = True
    for i in d:
      if (first):
        first = False
        if (charType == 's'):
          count = len(i)
        continue
      
      if (_getFormat(i) == charType):
        if (charType == 's'):
          count += len(i)
        else:
          count += 1
      else:
        fmtString += str(count) + charType
        charType  = _getFormat(i)
        if (charType == 's'):
          count = len(i)
        else:
          count = 1
  
    fmtString += str(count) + charType
       
  elif (type(data) is str):
    for l in range(len(data)):
      fmtString += _getFormat(data)

  else:
    fmtString = _getFormat(data)

  send = fmtString.encode()
  size = int((len(send) / b) + 1)
  message = struct.pack("I%ds" %len(send), size, send)

  userSocket.sendall(message)
  recvMsg = userSocket.recv(b)
  return fmtString

  
def defineSendFmt( formatStr, inpt, b=buffer):
  dataSize = { 'i':4 , 'f':4, 's':1, '?':1 }
  npy = inpt
  send = bytes()
  if (type(inpt) is list):
    inpt = flatten(inpt)
  else:
    return [formatStr], [[inpt]], send
    
  message = []
  format = []
  length = 0
  dataIndex = 0
  dataStart = 0
  fmtStart = 0
  fmtEnd = 1
  startMsg = []
  i = 0
  size = 0

  dim = array(npy).shape
  send += struct.pack('i', len(dim))
  size += dataSize['i']
  i += 1
    
  for k in dim:
    send += struct.pack('i', k)
    size += dataSize['i']
    fmtEnd += 1
    i += 1

  while (i < len(formatStr)):
    if (formatStr[i].isdigit()):
      length = 10 * length + int(formatStr[i])

    else:
      if (length == 0):
        length = 1

      size += length * dataSize[ formatStr[i] ]

      if (size > b):
        if (length > 1):
          firstPacket  = int( (length * dataSize[formatStr[i]] - (size - b)) / dataSize[formatStr[i]])

          if (formatStr[i] == 's'):
              message.append(flatten([inpt[dataStart:dataIndex], str(inpt[dataIndex][:firstPacket])]))
              inpt.insert(dataIndex+1, inpt[dataIndex][firstPacket:])
              dataStart = dataIndex+1
          else:
              message.append(flatten(inpt[dataStart:(dataIndex + firstPacket)]))
              dataStart = dataIndex + firstPacket
          size = 0
          dataIndex += firstPacket - length

          format.append( str(formatStr[fmtStart:fmtEnd] + str(firstPacket) + formatStr[i]))
          formatStr = str( str(length - firstPacket) + formatStr[i] + formatStr[i+1:])
          fmtStart = 0
          i = -1
          
        else:
          message.append(flatten(inpt[dataStart:dataIndex]))
          dataStart = dataIndex

          format.append( str(formatStr[fmtStart:i]))

          size = dataSize[formatStr[i]]
          fmtStart = i

      if ((formatStr[i] == 's')):
          dataIndex += 1
      else:
          dataIndex += length

      length = 0
      fmtEnd = i+1

    i += 1

  message.append(flatten(inpt[dataStart:]))
  format.append(str(formatStr[fmtStart:]))

  return format, message, send
  

def packNsend( format, message, send, b=buffer):
  for i in range(len(format)):
    for mes in message[i]:
      if (_getFormat(mes) == 's'):
        l = len(mes)
        mes = mes.encode()
        send += struct.pack("%ds" %l, mes)
      else:
        send += struct.pack(_getFormat(mes), mes)

    userSocket.sendall(send)
    recvMsg = userSocket.recv(b)
    send = bytes()

    
def sendData( data, bufferSize=buffer, showSendData=False):

  try:
    if (showSendData):
      print("\n\nBuffer Size: ", bufferSize, "\nSending: ")
      try:
        [print(r) for r in data]
      except:
        print(r)
      
    formatStr = sendFmtStr( data, b=bufferSize)
    fmt, sendData, begin = defineSendFmt( formatStr, data, b=bufferSize)
    if (showSendData):
      print("Final Format: ")
      [print(f) for f in fmt]
      print("Final Message: ")
      try:
        [print(m) for m in sendData]
      except:
        print(m)

    packNsend( fmt, sendData, begin, b=bufferSize)
  except BrokenPipeError:
    userSocket.close()
    sys.exit('Connection has been lost.')
  except ConnectionResetError:
    userSocket.close()
    sys.exit('Connection closed by peer.')
  except KeyboardInterrupt:
    userSocket.close()
    sys.exit('User terminated connection during send attempt.')


def sendMap( studentMap):
  if not(type(studentMap) is list) and not(type(studentMap[0]) is list):
    print("ERROR: Map must be of 2D list type.")
    return -1

  send = [zeros(len(studentMap[0])).tolist()]
  send[0][1] = 'Sample Header - Team_99'
  for row in studentMap:
    send.append(row)
  
  sendData( send)


def sendMapFile( studentMap, showData=False):
  mapLength = len(studentMap[-1])
  sendMap = []  

  for i in range(len(studentMap)):
    if (i < 6):
      temp = zeros(mapLength).tolist()
      temp = [int(t) for t in temp]
      if (i == 4):
        try:
          temp[0] = str(studentMap[i][0] + studentMap[i][1])
        except:
          temp[0] = studentMap[i][0]
      else:
        temp[0] = studentMap[i][0]
      sendMap.append(temp)
    else:
      if (len(studentMap[i]) != mapLength):
        print("MAP ERROR: Map data must have rows that are all the same length")
        return 0
      sendMap.append(studentMap[i])

  sendData( sendMap, showSendData=showData)
  return 1


def connect( host):
  global userSocket
  port = 12345 #Arbitrary, will be reassigned

  print('Attempting to connect to ', host)
  sys.stdout.flush()

  try:
    userSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    userSocket.connect((host, port))
  except KeyboardInterrupt:
    sys.exit("\nUser terminated connection attempt.")
    sys.exit("\nUser terminated connection attempt.")

  print('\r', '\bConnected to:  ', host)
  print('Press [ctrl + C] to stop\n')

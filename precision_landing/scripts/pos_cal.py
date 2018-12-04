import serial
import math
import time
import numpy as np
print "node for reading ranges from DWM1000 and calculate it to local coordinates"
simulate = True
#serIn =  serial.Serial('/dev/ttyUSB0',9600)  # open serial port
if simulate == False:
    serIn =  serial.Serial('/dev/tty0',9600)  # open serial port

numbOfActiveAnchors = 0
ActiveAnchors = []
anchor_id = []
AnchorRanges = [0]*3
newPos = []
"""
1.80,2.11,2.21, 1.59,0.85,0.06

1.80,2.13,2.21, 1.58,0.85,0.17
1.80,2.17,2.22, 1.55,0.86,0.31
1.83,2.20,2.26, 1.55,0.85,0.48
1.85,2.24,2.29, 1.53,0.85,0.59
1.87,2.27,2.29, 1.52,0.87,0.65
1.90,2.30,2.30, 1.52,0.88,0.72
"""
testData = ["100,1.80,-92","200,2.11,-92","100,1.80,-92","200,2.11,-92","100,1.80,-92","200,2.11,-92","300,2.11,-92"]
simIndex = 0
d= 3.52
i = d*0.5
j = d*(math.sqrt(3)/2)


print d , i , j

def calPos( anchors = [] ):
    newData = [None]*3    
    newData[0] = float(anchors[0])
    newData[1] = float(anchors[1])
    newData[2] = float(anchors[2])
    anchors = newData
    x =(anchors[0]**2-anchors[1]**2+d**2)/(2*d)
    y =((anchors[0]**2-anchors[2]**2+i**2+j**2)/(2*j))-(i/j*x)
    z = math.sqrt(abs(anchors[0]**2-x**2-y**2))   
    pos = [x,y,z]
    return pos

while True:#serIn.isOpen():
    if simulate == True:
        if simIndex > len(testData)-1:
            simIndex = 0
        line= testData[simIndex]
    else:
        line = serIn.readline()
    
    data= line.split(",")
    if len(data) == 3:
        anchor_id = int(data[0])
        anchor_range = float(data[1])
        if anchor_id not in ActiveAnchors:
            print "adding anchor"
            ActiveAnchors.append(anchor_id)
 
            AnchorRanges[(anchor_id/100)-1]=anchor_range
        elif anchor_id in ActiveAnchors:
            print "The ID is already in the mix" 
            del ActiveAnchors[:]
            print "adding anchor"
            ActiveAnchors.append(anchor_id)
 
            AnchorRanges[(anchor_id/100)-1]=anchor_range
 
        if  len(ActiveAnchors) == 3:
            print "Calculation new pos"
            newPos = calPos(AnchorRanges) 
            del ActiveAnchors[:]
            print newPos 
           
        if simulate == True:    
            time.sleep(1)
            simIndex = simIndex+1  


#serIn.close()
print "The serialport is closed"





    

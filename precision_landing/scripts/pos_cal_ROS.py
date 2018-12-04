#ROS node  to calculate pos from arduino ranges
#Eskild Andresen esand14 3/12/18


import rospy
import serial
import math
import time
import numpy as np
from precision_landing.msg import precland_sensor_data


# parameters
update_interval = 30
debug = False

class posCalROS(object): 
    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        self.simulate = False
        
        if self.simulate == False:
            self.serIn =  serial.Serial('/dev/ttyUSB0',9600)  # open serial port

        self.numbOfActiveAnchors = 0
        self.ActiveAnchors = []
        self.anchor_id = []
        self.AnchorRanges = [0]*3
        self.newPos = []
        """
        1.80,2.11,2.21, 1.59,0.85,0.06

        1.80,2.13,2.21, 1.58,0.85,0.17
        1.80,2.17,2.22, 1.55,0.86,0.31
        1.83,2.20,2.26, 1.55,0.85,0.48
        1.85,2.24,2.29, 1.53,0.85,0.59
        1.87,2.27,2.29, 1.52,0.87,0.65
        1.90,2.30,2.30, 1.52,0.88,0.72
        """
        self.testData = ["100,1.80,-92","200,2.11,-92","100,1.80,-92","200,2.11,-92","100,1.80,-92","200,2.11,-92","300,2.11,-92"]
        self.simIndex = 0
        self.d= 0.33#3.52
        self.i = self.d*0.5
        self.j = self.d*(math.sqrt(3)/2)

        self.sensor_data_pub    = rospy.Publisher("/landing/arduino_pos", precland_sensor_data, queue_size=0)


    def calPos(self , anchors = [] ):
        self.newData = [None]*3    
        self.newData[0] = float(anchors[0])
        self.newData[1] = float(anchors[1])
        self.newData[2] = float(anchors[2])
        anchors = self.newData
        x =(anchors[0]**2-anchors[1]**2+self.d**2)/(2*self.d)
        y =((anchors[0]**2-anchors[2]**2+self.i**2+self.j**2)/(2*self.j))-(self.i/self.j*x)
        z = math.sqrt(abs(anchors[0]**2-x**2-y**2))   
        self.pos = [x,y,z]
        return self.pos


    def run(self):
        if self.simulate == True:
            if self.simIndex > len(self.testData)-1:
                self.simIndex = 0
            line= self.testData[self.simIndex]
        else:
            line = self.serIn.readline()
        
        data= line.split(",")
        if len(data) == 3:
            self.anchor_id = int(data[0])
            self.anchor_range = float(data[1])
            if self.anchor_id not in self.ActiveAnchors:
                if debug:
                    print "adding anchor"
                self.ActiveAnchors.append(self.anchor_id)     
                self.AnchorRanges[(self.anchor_id/100)-1]=self.anchor_range
            elif self.anchor_id in self.ActiveAnchors:
                if debug:
                    print "The ID is already in the mix" 
                    print "adding anchor"
                
                del self.ActiveAnchors[:]
                self.ActiveAnchors.append(self.anchor_id)
     
                self.AnchorRanges[(self.anchor_id/100)-1]=self.anchor_range
     
            if  len(self.ActiveAnchors) == 3:
                if debug:
                    print "Calculation new pos"
                newPos = self.calPos(self.AnchorRanges) 
                del self.ActiveAnchors[:]
                if debug:
                    print newPos 
                sensor_data = precland_sensor_data(newPos[0], newPos[1], newPos[2])
                self.sensor_data_pub.publish(sensor_data)
               
            if self.simulate == True:    
                #time.sleep(1)
                self.simIndex = self.simIndex+1   


if __name__ == "__main__":

    rospy.init_node('pos_cal')
    rospy.sleep(1)

    pc = posCalROS()

    while not rospy.is_shutdown():
        pc.run()
        pc.rate.sleep()


#!/usr/bin/env python3
import rospy
import time
import math
import rospkg
import numpy as np
import rmsd

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
#from node_monitor.msg import heartbeat
from precision_landing.msg import precland_sensor_data

import threading
import matplotlib.cm as cm


# defines

# parameters
update_interval = 15







class Graph(object):

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        rospack = rospkg.RosPack()


        rospy.Subscriber("/landing/arduino_pos", precland_sensor_data, self.on_arduino_pos)
        rospy.Subscriber("/telemetry/set_landing_target", telemetry_landing_target, self.on_set_landing_target)

        plt.axis([-5, 7, -5, 9])

        self.arduino_pos_x = np.array([])
        self.arduino_pos_y = np.array([])

        self.set_landing_target_x = np.array([])
        self.set_landing_target_y = np.array([])

        self.protection = threading.Lock()


        plt.show(block=False)


    def on_arduino_pos(self, msg):
        print(msg)
        (x,y,z) = (msg.y,msg.x,-msg.z)

        with self.protection:
            self.arduino_pos_x = np.concatenate([self.arduino_pos_x, [x]])
            self.arduino_pos_y = np.concatenate([self.arduino_pos_y, [y]])

            if len(self.arduino_pos_x) > 10:
                self.arduino_pos_x = np.delete(self.arduino_pos_x, 0)
                self.arduino_pos_y = np.delete(self.arduino_pos_y, 0)



    def on_set_landing_target(self, msg):
        #(x,y,z) = (msg.x,msg.y,msg.z)

        #self.set_landing_target_x.append(x)
        #self.set_landing_target_y.append(y)

        pass



    def run(self):
        print("Hello")
        plt.clf()
        # update coordinate axis, make it sliding:
        try:
            axisMargin = 1
            xLimMin = np.amin(self.arduino_pos_x) - axisMargin
            xLimMax = np.amax(self.arduino_pos_x) + axisMargin
            yLimMin = np.amin(self.arduino_pos_y) - axisMargin
            yLimMax = np.amax(self.arduino_pos_y) + axisMargin

            limMin = min(xLimMin, yLimMin)
            limMax = max(xLimMax, yLimMax)
            
            plt.axis([limMin, limMax, limMin,limMax])
        except Exception:
            pass


        
        with self.protection:
            colors = cm.rainbow(np.linspace(0, 1, len(self.arduino_pos_x)))
            plt.scatter(self.arduino_pos_x,self.arduino_pos_y, color=colors, marker="x")


        # Plot the Triangle:

        p1 = [0.,0.]
        p2 = [0., 3.52]
        p3 = [3.52*math.sqrt(3)/2.0, 1.76]


        # Plot the Anchors:

        plt.plot(p1[0], p1[1], 'bo')
        plt.text(p1[0]-0.3, p1[1]-0.3, "anchor1", fontsize=9)
        plt.plot(p2[0], p2[1], 'bo')
        plt.text(p2[0]-0.3, p2[1]-0.3, "anchor2", fontsize=9)
        plt.plot(p3[0], p3[1], 'bo')
        plt.text(p3[0]+0.2, p3[1]+0.2, "anchor3", fontsize=9)

        newline(p1,p2)
        newline(p2,p3)
        newline(p3,p1)



        plt.grid()
        plt.draw()
        plt.axis('equal')
        plt.pause(0.05)

        #for i in range(len(self.set_landing_target_x)):
        #    plt.scatter(self.set_landing_target_x[i], self.set_landing_target_y[i])
        #    plt.draw()
        #    plt.pause(0.05)
        #plt.show()



        #for i in range(len()):
            #plt.scatter(self.arduino_pos_x[i], self.self.arduino_pos_y[i])

            

        #plt.show()
        #print("Heellooo")
        pass

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")




def newline(p1, p2):
    ax = plt.gca()
    xmin, xmax = ax.get_xbound()

    if(p2[0] == p1[0]):
        xmin = xmax = p1[0]
        ymin, ymax = ax.get_ybound()
    else:
        ymax = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmax-p1[0])
        ymin = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmin-p1[0])

    l = mlines.Line2D([xmin,xmax], [ymin,ymax])
    ax.add_line(l)
    return l



if __name__ == "__main__":
    rospy.init_node('plotting')
    rospy.sleep(1)

    graph = Graph()

    rospy.on_shutdown(graph.shutdownHandler)
    # rospy.spin()



    while not rospy.is_shutdown():
        graph.run()
        graph.rate.sleep()

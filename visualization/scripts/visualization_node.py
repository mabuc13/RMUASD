#!/usr/bin/env python3
import rospy
import time

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from precision_landing.msg import precland_sensor_data
from node_monitor.msg import heartbeat

from gcs.msg import GPS, DroneInfo

from coordinate import Coordinate

from utm_parser.srv import *
from utm_parser.msg import *

import sys
import numpy as np

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import cv2




# This node should visualize a few things
#   - The Drone's current position
#   - The path the drone is going to take
#   - Static no flight zones
#   - Dynamic no flight zones





# defines

# parameters
update_interval = 10




class Visualization(object): 

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        self.coordinate = None
        self.snfzMap = None
        self.finalMap = None

        self.wantSNFZMAP = True


        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/drone_handler/DroneInfo", DroneInfo, self.on_drone_info)

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

    def on_drone_info(self, msg):
        self.coordinate = Coordinate(GPS_data=msg.position)


        if self.wantSNFZMAP:
                self.snfzMap = self.make_static_map(self.coordinate)
                print("Updated SNFZ Map!")
                # try to make it visibile :)
                
                #print(self.snfzMap)
                print(self.snfzMap.snfz_map[0])
                print("Length: ", len(self.snfzMap.snfz_map[0].row))
                print("#######")
                print(self.snfzMap.snfz_map[0])
                print(type(self.snfzMap.snfz_map[0]))
                print("#######")
                print(type(self.snfzMap)) 
                print("Done")

                print("Length of the map: ", len(self.snfzMap.snfz_map))
                print("Resolution: ", self.snfzMap.resolution)
                #print("Coordinate_lower_left: ", self.snfzMap.coordinate_lower_left)
                #print("Coordinate_upper_right ", self.snfzMap.coordinate_upper_right)


                print("Rows: ", len(self.snfzMap.snfz_map))
                print("Cols: ", len(self.snfzMap.snfz_map[0].row))
                drawMap = np.zeros((len(self.snfzMap.snfz_map),len(self.snfzMap.snfz_map[0].row)))
                
                for row in range(len(self.snfzMap.snfz_map)):
                    for col in range(len(self.snfzMap.snfz_map[row].row)):
                        drawMap[row,col] = self.snfzMap.snfz_map[row].row[col]


                self.finalMap = drawMap
                self.wantSNFZMAP = False



        #print(self.coordinate.str(), " :)")
        #print(self.snfzMap)


    def on_heartbeat_status(self, msg):
        pass

    def run(self):        
        if self.finalMap is not None:
            self.show_map()

        pass


    def make_static_map(self, start, map_padding=250):

        lower_left = Coordinate(easting=start.easting - map_padding, northing=start.northing - map_padding)
        upper_right = Coordinate(easting=start.easting + map_padding, northing=start.northing + map_padding)

        print("[Path planner]: " + "Waiting for UTM")
        rospy.wait_for_service('/utm_parser/get_snfz')
        get_snfz_handle = rospy.ServiceProxy('/utm_parser/get_snfz', get_snfz)

        map = get_snfz_handle(lower_left.GPS_data, upper_right.GPS_data)
        return map

    def show_map(self):
        drawMapResized = cv2.resize(self.finalMap, (1000, 1000)) 
        print("Updating Map")
        cv2.imshow("BW Image",drawMapResized)

        #cv2.waitKey(1)
        cv2.waitKey(10)
        #cv2.destroyAllWindows()



if __name__ == "__main__":
    rospy.init_node('visualization_node')
    rospy.sleep(1)

    v = Visualization()

    rospy.on_shutdown(v.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'precision_landing'
    heart_msg.rate = update_interval

    while not rospy.is_shutdown():
        v.run()

        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        v.rate.sleep()

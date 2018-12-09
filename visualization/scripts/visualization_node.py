#!/usr/bin/env python3
import rospy
import time

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from precision_landing.msg import precland_sensor_data
from node_monitor.msg import heartbeat
from std_msgs.msg import String



from shapely import geometry, wkt

from gcs.msg import GPS, DroneInfo, DronePath

from coordinate import Coordinate

from utm_parser.srv import *
from utm_parser.msg import *

import sys
import numpy as np

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import json
import cv2

import threading




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
        self.final_snfz_map = None
        self.final_dnfz_map = None
        self.start = None
        self.final_currentPosition_map = None
        self.final_path_map = None

        self.total_rows = None
        self.total_cols = None


        self.protection = threading.Lock()

        self.wantSNFZMAP = True


        self.dynamic_no_flight_zones = {}

        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/drone_handler/DroneInfo", DroneInfo, self.on_drone_info)
        rospy.Subscriber("/utm/dynamic_no_fly_zones", String, self.on_dynamic_no_fly_zones)
        rospy.Subscriber("/gcs/forwardPath", DronePath, self.on_drone_path)

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")


    def on_drone_path(self, msg):
        self.final_path_map = np.zeros((self.total_rows, self.total_cols))
        print("New drone Path!")
        i = 0
        for position in msg.Path:
            path_part_coordinate = Coordinate(GPS_data=position)

            (east, north) = self.getPixelCoordinate(path_part_coordinate)

            with self.protection:
                cv2.circle(self.final_path_map,(east, north), 10, 1.0, -1)
                cv2.putText(self.final_path_map,"path"+str(i), (east - 75, north + 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), thickness=2)
                i += 1


    def getPixelCoordinate(self, coord):
        east = int(1500 + (coord.easting-self.start.easting) * 3000./500.)
        north = int(1500 - (coord.northing-self.start.northing) * 3000./500.)

        return (east, north)

    def get_dnfz_client(self):
        rospy.wait_for_service('/utm_parser/get_dnfz')
        self.get_dnfz_handle = rospy.ServiceProxy('/utm_parser/get_dnfz', get_dnfz)
        dnfz_string = self.get_dnfz_handle()
        #print dnfz_string.dnfz
        obj = String
        obj.data = dnfz_string.dnfz
        self.on_dynamic_no_fly_zones(obj)


    def on_dynamic_no_fly_zones(self, msg):

        '''
        Callback function
        Use "int_id" in the string as key in the dict of dnfz
        '''
        try:
            all_json_objs = json.loads(str(msg.data))
            for json_obj in all_json_objs:
                #print("item:", json_obj)
                # Update if it is alredy known
                if json_obj["int_id"] in self.dynamic_no_flight_zones:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                else:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
        except ValueError:
            print("Visualization: Couldn't convert string from UTM server to json..")

        # let's see what we have inside 

        #self.start.easting self.start.northing
        #print("Start e, n:", self.start.easting, self.start.northing)
        #print("Start lat, lon", self.start.lat, self.start.lon)

    def on_drone_info(self, msg):        
        self.coordinate = Coordinate(GPS_data=msg.position)



        if self.wantSNFZMAP:
            self.create_snfz_map()
            self.wantSNFZMAP = False

            
            # get already existing dynamic no flight zones
            self.get_dnfz_client()

        (east, north) = self.getPixelCoordinate(self.coordinate)

        with self.protection:
            # draw current position
            if self.total_rows is not None:
                self.final_currentPosition_map = np.zeros((self.total_rows, self.total_cols))
            else:
                return
            
            cv2.circle(self.final_currentPosition_map,(east, north), 20, 1.0,-1)
            cv2.putText(self.final_currentPosition_map, "current position", (east - 100 , north + 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), thickness=2)

        #print(self.coordinate.str(), " :)")
        #print(self.snfzMap)


    def on_heartbeat_status(self, msg):
        pass

    def run(self):        
        if self.final_snfz_map is not None:
            self.update_dnfz()
            self.show_map()
            self.final_dnfz_map = np.zeros((self.total_rows, self.total_cols))
            
            
            pass
        pass

    def update_dnfz(self):
         # Update the dynamic no flight zones in the final map!

        currentTime = int(time.time())
        for key,val in self.dynamic_no_flight_zones.items():
            #print(key, "=>", val)

            if int(val["valid_from_epoch"]) > currentTime or int(val["valid_to_epoch"]) < currentTime:
                # then don't plot it
                #print("Discard dnfz - not active right now!")
                continue

            if val["geometry"] == "polygon":
                #print("drawing a polygon")

                # first, draw a point:

                # coord[0] --> easting
                # coord[1] --> northing
                #print(list(val["polygon"].exterior.coords))



                coordinateString = val["coordinates"]
                coordinateArray = coordinateString.split(' ')

                first = True

                for coordinatePair in coordinateArray:
                    coord = coordinatePair.split(',')


                    (east, north) = self.getPixelCoordinate(Coordinate(lat=coord[1], lon=coord[0]))
                    
                    toAppend = np.array([[east, north]])

                    if first:
                        pts = np.array(toAppend)
                        #print("Polygon here:", pts)
                        first = False
                    else:
                        pts = np.concatenate([pts, toAppend])

                    
                pts = pts.reshape((-1,1,2))
                #print(pts)

                with self.protection:
                    cv2.polylines(self.final_dnfz_map,[pts],True,1.0,10)

            elif val["geometry"] == "circle":
                #print("drawing a circle")
                
                coordinateString = val["coordinates"]
                coordinateArray = coordinateString.split(',')
                
                (east, north) = self.getPixelCoordinate(Coordinate(lon=coordinateArray[0], lat=coordinateArray[1]))
                #print("Circle here:", (east,north))
                with self.protection:
                    cv2.circle(self.final_dnfz_map,(east, north), int(coordinateArray[2]), 1.0, 2)


    def make_static_map(self, start, map_padding=250):

        lower_left = Coordinate(easting=start.easting - map_padding, northing=start.northing - map_padding)
        upper_right = Coordinate(easting=start.easting + map_padding, northing=start.northing + map_padding)

        print("[Path planner]: " + "Waiting for UTM")
        rospy.wait_for_service('/utm_parser/get_snfz')
        get_snfz_handle = rospy.ServiceProxy('/utm_parser/get_snfz', get_snfz)

        map = get_snfz_handle(lower_left.GPS_data, upper_right.GPS_data)
        return map

    def create_snfz_map(self):
        self.start = self.coordinate
        self.snfzMap = self.make_static_map(self.start)
        print("Updated SNFZ Map!")
        # try to make it visibile :)
        
        # print(self.snfzMap)
        # print(self.snfzMap.snfz_map[0])
        # print("Length: ", len(self.snfzMap.snfz_map[0].row))
        # print("#######")
        # print(self.snfzMap.snfz_map[0])
        # print(type(self.snfzMap.snfz_map[0]))
        # print("#######")
        # print(type(self.snfzMap)) 
        # print("Done")

        #print("Length of the map: ", len(self.snfzMap.snfz_map))
        print("Resolution: ", self.snfzMap.resolution)
        #print("Coordinate_lower_left: ", self.snfzMap.coordinate_lower_left)
        #print("Coordinate_upper_right ", self.snfzMap.coordinate_upper_right)


        self.total_rows = len(self.snfzMap.snfz_map)
        self.total_cols = len(self.snfzMap.snfz_map[0].row)
        print("Rows: ", self.total_rows)
        print("Cols: ", self.total_cols)
        drawMap = np.zeros((self.total_rows, self.total_cols))
        
        for row in range(len(self.snfzMap.snfz_map)):
            for col in range(len(self.snfzMap.snfz_map[row].row)):
                drawMap[row,col] = self.snfzMap.snfz_map[row].row[col]
                # if int(drawMap[row,col]) is not int(0):
                #     print(drawMap[row,col])

        self.final_snfz_map = drawMap
        self.final_dnfz_map = np.zeros((self.total_rows, self.total_cols))
        self.final_currentPosition_map = np.zeros((self.total_rows, self.total_cols))
        self.final_path_map = np.zeros((self.total_rows, self.total_cols))




    def show_map(self):
        with self.protection:

            self.finalMap = self.final_snfz_map + self.final_dnfz_map + self.final_currentPosition_map + self.final_path_map
            self.finalMap = np.clip(self.finalMap, 0.0, 1.0)

            drawMapResized = cv2.resize(self.finalMap, (500, 500)) 
            #print("Updating Map")
            cv2.imshow("Map of SNFZ and DNFZ",drawMapResized)

            #cv2.waitKey(1)
            cv2.waitKey(5)
            #cv2.destroyAllWindows()



if __name__ == "__main__":
    rospy.init_node('visualization_node')
    rospy.sleep(1)

    v = Visualization()

    rospy.on_shutdown(v.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'visualization'
    heart_msg.rate = update_interval

    while not rospy.is_shutdown():
        v.run()

        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        v.rate.sleep()

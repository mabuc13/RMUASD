#!/usr/bin/env python

import numpy as np
from coordinate import Coordinate
from AStar_1 import astar
from exportkml import kmlclass
import rospy
from gcs.msg import GPS, DronePath


coord = GPS(longitude=,
            latitude=,
            altidute=)

msg = DronePath(Path=)


class PathPlanner(object):
    def __init__(self, start=Coordinate(lat=55.470415, lon=10.329449), goal=Coordinate(lat=55.470415, lon=10.329449)):
        self.start = Coordinate(lat=start.latitude, lon=start.longitude)
        self.goal = Coordinate(lat=goal.latitude, lon=goal.longitude)

        # Default position at HCA Airport: lat=55.470415, lon=10.329449
        self.global_bottom_left = Coordinate(lat=55.470415, lon=10.329449)
        self.global_top_right = Coordinate(lat=55.470415, lon=10.329449)
        self.global_map_width = 0
        self.global_map_height = 0
        self.path = []
        self.map_padding = 100  # padding on top, bottom, left and right of map
        self.map_zeros = np.zeros((self.global_map_height, self.global_map_width), dtype=np.int)

        self.set_global_coordinates()

        # Ros services:
        self.service_get_path = rospy.Service("/pathplan/get_path", GetPath, self.get_path())

    def set_global_coordinates(self):
        if self.start.northing < self.goal.northing:
            if self.start.easting < self.goal.easting:
                self.global_bottom_left.northing = self.start.northing - self.map_padding
                self.global_bottom_left.easting = self.start.easting - self.map_padding
                self.global_top_right.northing = self.goal.northing + self.map_padding
                self.global_top_right.easting = self.goal.easting + self.map_padding
            else:
                self.global_bottom_left.northing = self.start.northing - self.map_padding
                self.global_bottom_left.easting = self.goal.easting - self.map_padding
                self.global_top_right.northing = self.goal.northing + self.map_padding
                self.global_top_right.easting = self.start.easting + self.map_padding
        else:
            if self.start.easting < self.goal.easting:
                self.global_bottom_left.northing = self.goal.northing - self.map_padding
                self.global_bottom_left.easting = self.start.easting - self.map_padding
                self.global_top_right.northing = self.start.northing + self.map_padding
                self.global_top_right.easting = self.goal.easting + self.map_padding
            else:
                self.global_bottom_left.northing = self.goal.northing - self.map_padding
                self.global_bottom_left.easting = self.goal.easting - self.map_padding
                self.global_top_right.northing = self.start.northing + self.map_padding
                self.global_top_right.easting = self.start.easting + self.map_padding

        self.global_bottom_left.update_geo_coordinates()
        self.global_top_right.update_geo_coordinates()

        self.global_map_width = int(self.global_top_right.easting - self.global_bottom_left.easting)
        self.global_map_height = int(self.global_top_right.northing - self.global_bottom_left.northing)

        self.map_zeros = np.zeros((self.global_map_height, self.global_map_width), dtype=np.int)

    def set_map_padding(self, new_padding):
        self.map_padding = new_padding
        self.set_global_coordinates()

    def set_start_and_goal(self, start, goal):
        self.start = Coordinate(lat=start.latitude, lon=start.longitude)
        self.goal = Coordinate(lat=goal.latitude, lon=goal.longitude)

    def compute_path(self):
        rel_start_pos = (int(self.start.northing - self.global_bottom_left.northing),
                         int(self.start.easting - self.global_bottom_left.easting))
        rel_goal_pos = (int(self.goal.northing - self.global_bottom_left.northing),
                        int(self.goal.easting - self.global_bottom_left.easting))
        rel_path = astar(self.map_zeros, rel_start_pos, rel_goal_pos)

        self.path.clear()

        #TODO: Convert this to the GPS format that Kasper made
        self.path.append(self.start)
        for j in reversed(rel_path):
            self.path.append(Coordinate(northing=(j[0] + self.global_bottom_left.northing),
                                        easting=(j[1] + self.global_bottom_left.easting)))
        self.path.append(self.goal)

    def export_kml_path(self, name):
        # width: defines the line width, use e.g. 0.1 - 1.0
        kml = kmlclass()
        kml.begin(name+'klm', 'Example', 'Example on the use of kmlclass', 0.1)
        # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
        # altitude: use 'absolute' or 'relativeToGround'
        kml.trksegbegin('', '', 'red', 'absolute')
        for i in self.path:
            kml.trkpt(i.lat, i.lon, 0.0)
        kml.trksegend()
        kml.end()

    def get_path(self):
        self.compute_path()
        return self.path

if __name__ == '__main__':
    rospy.init_node('pathplan')
    rospy.sleep(1)

    drone_path_planner = PathPlanner()

    rospy.spin()


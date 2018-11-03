#!/usr/bin/env python

import numpy as np
from coordinate import Coordinate
from AStar_1 import astar
from exportkml import kmlclass
import rospy
from gcs.msg import *
from gcs.srv import *
from Path_simplifier import PathSimplifier
from eta_estimator import *
import simplifier_rmuast


class PathPlanner(object):
    def __init__(self, start=Coordinate(lat=55.470415, lon=10.329449), goal=Coordinate(lat=55.470415, lon=10.329449)):
        self.start = Coordinate(GPS_data=start.GPS_data)
        self.goal = Coordinate(GPS_data=goal.GPS_data)

        # Default position at HCA Airport: lat=55.470415, lon=10.329449
        self.global_bottom_left = start
        self.global_top_right = goal
        self.global_map_width = 0
        self.global_map_height = 0
        self.path = []
        self.map_padding = 100  # padding on top, bottom, left and right of map
        self.map_zeros = np.zeros((self.global_map_height, self.global_map_width), dtype=np.int)

        self.rmuast_simplifier = simplifier_rmuast.FlightPlanner()

        self.set_global_coordinates()
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
        self.path = []

        #TODO: Convert this to the GPS format that Kasper made
        self.path.append(self.start)
        for j in reversed(rel_path):
            self.path.append(Coordinate(northing=(j[0] + self.global_bottom_left.northing),
                                        easting=(j[1] + self.global_bottom_left.easting)))
        self.path.append(self.goal)
        
        # Simplify path:
        # self.export_kml_path("not_simple")

        # self.rmuast_simplifier.loadPath(self.path)

        # self.rmuast_simplifier.simplifyFlightPlan(dist=0.1)
        # self.path = self.rmuast_simplifier.getSimpleCoordinates()
        # self.export_kml_path("simple")
        ps = PathSimplifier(self.path, step_size=20)
        ps.delete_with_step_size_safe()
        self.path = ps.get_simple_path()
    def export_kml_path(self, name):
        print("Exporting")
        # width: defines the line width, use e.g. 0.1 - 1.0
        kml = kmlclass()
        kml.begin(name+'.kml', 'Example', 'Example on the use of kmlclass', 0.1)
        # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
        # altitude: use 'absolute' or 'relativeToGround'
        kml.trksegbegin('', '', 'red', 'absolute')
        for i in self.path:
            kml.trkpt(i.lat, i.lon, 0.0)
        kml.trksegend()
        kml.end()


def handle_getPathPlan(req):
    print("[Path planner]: "+"Planning from: lon("+ str(req.start.longitude)+"), lat("+ str(req.start.latitude)+"), alt(" + str(req.start.altitude) + ") to lon("+ str(req.start.longitude)+"), lat("+ str(req.start.latitude)+"), alt(" + str(req.start.altitude)+")")
    start = Coordinate(GPS_data=req.start)
    theend = Coordinate(GPS_data=req.end)
    planner = PathPlanner(start=start,goal=theend)
    planner.compute_path()
    plan = planner.path
    GPSPlan = []
    for point in plan:
        GPSPlan.append(point.GPS_data)
    return pathPlanResponse(GPSPlan)

def handle_ETA(req):
    print("[Path planner]: Calculating ETA")
    obj = eta()
    etaT = obj.eta_estimate(req.path,req.speed)

    return getEtaResponse(etaT)

def handle_distanceCalculations(req):
    print("[Path planner]: Calculating distance")
    obj = eta()
    distance = obj.distance_between_positions(req.point1,req.point2)
    return gps2distanceResponse(distance)


if __name__ == '__main__':
    rospy.init_node('pathplan')
    rospy.sleep(1)

    s = rospy.Service('pathplan/getPlan',pathPlan, handle_getPathPlan)
    s2= rospy.Service('pathplan/getEta',getEta,handle_ETA)
    s3= rospy.Service('pathplan/GPS2GPSdist',gps2distance,handle_distanceCalculations)
    print("[Path planner]: "+"Path planner running")
    rospy.spin()

#!/usr/bin/env python

import numpy as np
from coordinate import Coordinate
from AStar_2 import astar
from exportkml import kmlclass
import rospy
from gcs.msg import *
from gcs.srv import *
from Path_simplifier import PathSimplifier
from eta_estimator import *
import time
import simplifier_rmuast


class PathPlanner(object):
    def __init__(self, start=Coordinate(lat=55.470415, lon=10.329449), goal=Coordinate(lat=55.470415, lon=10.329449)):
        # Default position at HCA Airport: lat=55.470415, lon=10.329449
        self.start = Coordinate(GPS_data=start.GPS_data)
        self.goal = Coordinate(GPS_data=goal.GPS_data)
        self.path = []

        self.rmuast_simplifier = simplifier_rmuast.FlightPlanner()

    def set_start_and_goal(self, start, goal):
        self.start = Coordinate(lat=start.latitude, lon=start.longitude)
        self.goal = Coordinate(lat=goal.latitude, lon=goal.longitude)

    def compute_path(self):
        print("Distance: ",
              sqrt((self.start.easting - self.goal.easting) ** 2 + (self.start.northing - self.goal.northing) ** 2))
        print("Computing path...")
        t0 = time.time()
        path_reversed = astar(self.start, self.goal, step_multiplier=8)
        t1 = time.time()
        print("Found a path in %s seconds." % (t1 - t0))

        self.path = []
        self.path.append(self.start)
        for j in reversed(path_reversed):
            self.path.append(j)
        self.path.pop(-1)
        self.path.append(self.goal)

        print("Number of waypoints: " + str(len(self.path)))
        # Simplify path:
        ps = PathSimplifier(self.path, step_size=16)
        ps.delete_with_step_size_safe(threshold=8)
        self.path = ps.get_simple_path()
        print("Number of waypoints after simplifier: " + str(len(self.path)))

    def export_kml_path(self, name):
        print("Exporting")
        # width: defines the line width, use e.g. 0.1 - 1.0
        kml = kmlclass()
        kml.begin(name+'.kml', 'Example', 'Example on the use of kmlclass', 0.1)
        # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
        # altitude: use 'absolute' or 'relativeToGround'
        kml.trksegbegin('', '', 'blue', 'absolute')
        for i in self.path:
            kml.trkpt(i.lat, i.lon, 0.0)
        kml.trksegend()
        kml.end()


def handle_getPathPlan(req):
    print("[Path planner]: "+"Planning from: lon("+ str(req.start.longitude)+"), lat("+ str(req.start.latitude)+"), alt(" + str(req.start.altitude) +
          ") to lon("+ str(req.start.longitude)+"), lat("+ str(req.start.latitude)+"), alt(" + str(req.start.altitude)+")")
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
    """
    ll = GPS()
    ll.latitude = 55.425762
    ll.longitude = 10.379273
    ll.altitude = 0
    #55.425762, 10.379273
    #55.425013, 10.380078
    ur = GPS()
    ur.latitude = 55.425013
    ur.longitude = 10.39517

    #55.424200, 10.39517
    ur.altitude = 0.0

    start = Coordinate(lat=55.437022, lon=10.412464)
    end = Coordinate(lat=55.431777, lon=10.443342)

    planner = PathPlanner(start=start, goal=end)
    planner.compute_path()
    plan = planner.path

    planner.export_kml_path("Test_plan")
    """
    print("[Path planner]: "+"Path planner running")
    rospy.spin()

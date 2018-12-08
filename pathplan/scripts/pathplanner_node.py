#!/usr/bin/env python

import numpy as np
from coordinate import Coordinate
from AStar_1 import AStar
from exportkml import kmlclass
import rospy
from gcs.msg import *
from gcs.srv import *
from Path_simplifier import PathSimplifier
from eta_estimator import *
import time
import simplifier_rmuast
from node_monitor.msg import *

from utm_parser.srv import *
from utm_parser.msg import *


class PathPlanner(object):
    def __init__(self, start,  goal, map):
        #self, start=Coordinate(lat=55.470415, lon=10.329449), goal=Coordinate(lat=55.470415, lon=10.329449)
        # Default position at HCA Airport: lat=55.470415, lon=10.329449
        self.start = Coordinate(GPS_data=start.GPS_data)
        self.goal = Coordinate(GPS_data=goal.GPS_data)
        self.path = []
        self.map = map

        self.rmuast_simplifier = simplifier_rmuast.FlightPlanner()

    def set_start_and_goal(self, start, goal):
        self.start = Coordinate(lat=start.latitude, lon=start.longitude)
        self.goal = Coordinate(lat=goal.latitude, lon=goal.longitude)

    def compute_path(self, dynamic, ground_speed=5, start_time=0, map_padding=2500):

        print("[Path planner]: "+"Distance: ",
              sqrt((self.start.easting - self.goal.easting) ** 2 + (self.start.northing - self.goal.northing) ** 2))
        print("[Path planner]: "+"Computing path...")

        t0 = time.time()
        astar_object = None
        if dynamic:
            astar_object = AStar(self.start, self.goal, self.map, map_padding,step_multiplier=4)
        else:
            astar_object = AStar(self.start, self.goal, self.map, map_padding,step_multiplier=8)

        astar_object.set_start_and_goal(self.start, self.goal, start_time)
        #TODO request dynamic no fly zones from utm parser
        path_reversed = astar_object.compute_astar(dynamic, ground_speed) #True equals dynamic no flight zone

        if path_reversed == False:
            print "[Path planner]: No path found"
            self.path = []
        else:

            #path_reversed = astar(self.start, self.goal, self.map, map_padding,step_multiplier=8)
            t1 = time.time()

            print("[Path planner]: "+"Found a path in %s seconds." % (t1 - t0))

            self.path = []
            self.path.append(self.start)
            #print "Path_reversed",path_reversed
            for j in reversed(path_reversed):
                self.path.append(j)
            self.path.pop(-1)
            self.path.append(self.goal)

            print("[Path planner]: "+"Number of waypoints: " + str(len(self.path)))
            # Simplify path:

            if(len(self.path) > 2):
                self.rmuast_simplifier.loadPath(self.path)
                self.rmuast_simplifier.simplifyByDistance(3)
                self.path = self.rmuast_simplifier.getSimpleCoordinates()
                # if dynamic:
                #     ps = PathSimplifier(self.path, step_size=2)
                #     ps.delete_with_step_size_safe(threshold=2)
                # else:
                #     ps = PathSimplifier(self.path, step_size=4)
                #     ps.delete_with_step_size_safe(threshold=8)

                # self.path = ps.get_simple_path()
                print("[Path planner]: "+"Number of waypoints after simplifier: " + str(len(self.path)))

    def export_kml_path(self, name):
        print("[Path planner]: "+"Exporting")
        # width: defines the line width, use e.g. 0.1 - 1.0
        kml = kmlclass()
        name = "../"+name
        kml.begin(name+'.kml', 'Example', 'Example on the use of kmlclass', 0.1)
        # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
        # altitude: use 'absolute' or 'relativeToGround'
        kml.trksegbegin('', '', 'blue', 'absolute')
        for i in self.path:
            kml.trkpt(i.lat, i.lon, 0.0)
        kml.trksegend()
        kml.end()


path_planner_dict = {}
plannum = 0
def handle_getPathPlan(req): #TODO add boolean for dynamic and start time in the requested message

    global path_planner_dict,plannum
    start = Coordinate(GPS_data=req.start)
    theend = Coordinate(GPS_data=req.end)
    map_padding = 2500
    if req.drone_id in path_planner_dict:
        map, old_start = path_planner_dict[req.drone_id]

        if (old_start.easting - map_padding > start.easting or old_start.northing - map_padding > start.northing or
                old_start.easting + map_padding < start.easting or old_start.northing + map_padding < start.northing):

            map = make_static_map(start)
            path_planner_dict[req.drone_id] = map, start
    else:
        map = make_static_map(start)
        path_planner_dict[req.drone_id] = map, start


    map, old_start = path_planner_dict[req.drone_id]

    print("[Path planner]: Planning from: " + start.str() + " - to: " + theend.str())
    planner = PathPlanner(start, theend, map)
    planner.compute_path(req.useDNFZ, req.velocity, req.startTime, map_padding)
    plan = planner.path

    #planner.export_kml_path("Path"+str(plannum))
    plannum = plannum +1
    
    GPSPlan = []
    for point in plan:
        GPSPlan.append(point.GPS_data)
    return pathPlanResponse(GPSPlan)



def make_static_map(start, map_padding=2500):

    lower_left = Coordinate(easting=start.easting - map_padding, northing=start.northing - map_padding)
    upper_right = Coordinate(easting=start.easting + map_padding, northing=start.northing + map_padding)

    print("[Path planner]: " + "Waiting for UTM")
    rospy.wait_for_service('/utm_parser/get_snfz')
    get_snfz_handle = rospy.ServiceProxy('/utm_parser/get_snfz', get_snfz)

    map = get_snfz_handle(lower_left.GPS_data, upper_right.GPS_data)
    return map

def handle_ETA(req):
    # print("[Path planner]: Calculating ETA")
    obj = eta()
    etaT = obj.eta_estimate(req.path,req.speed)

    return getEtaResponse(etaT)

def handle_distanceCalculations(req):
    #print("[Path planner]: Calculating distance")
    obj = eta()
    distance = obj.distance_between_positions(req.point1,req.point2)
    return gps2distanceResponse(distance)

if __name__ == '__main__':
    rospy.init_node('pathplan')
    rospy.sleep(1)


    s = rospy.Service('pathplan/getPlan',pathPlan, handle_getPathPlan)
    s2= rospy.Service('pathplan/getEta',getEta,handle_ETA)
    s3= rospy.Service('pathplan/GPS2GPSdist',gps2distance,handle_distanceCalculations)
    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)



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
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'pathplan'
    heart_msg.rate = 1

    while not rospy.is_shutdown():
        rospy.Rate(heart_msg.rate).sleep()
        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)

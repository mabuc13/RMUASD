#!/usr/bin/env python
# Based on:
# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python (2.7)
# Please give credit if used

from heapq import *
from math import sqrt
from coordinate import Coordinate
#import numpy as np
import rospy
import time
#import json
import string
from shapely import geometry
from std_msgs.msg import String
import string
from utm_parser.srv import *
import numpy as np
import json
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt

#from bokeh.plotting import figure, output_file, show
#from utm_parser.srv import *
#from utm_parser.msg import *


class AStar:
    def __init__(self, coord_start, coord_goal, static_map, map_padding, step_multiplier=1):
        self.start = (coord_start.easting, coord_start.northing, 0)
        self.goal = (coord_goal.easting, coord_goal.northing, 0)
        self.map = static_map
        self.map_res = static_map.resolution
        self.map_padding = map_padding
        self.map_image = np.zeros((static_map.map_width, static_map.map_height, 1), np.uint8)
        self.map_width = static_map.map_width
        self.map_height = static_map.map_height
        self.lower_left = Coordinate(easting=coord_start.easting - map_padding,
                                     northing=coord_start.northing - map_padding)
        self.upper_right = Coordinate(easting=coord_start.easting + map_padding,
                                      northing=coord_start.northing + map_padding)
        self.step_multiplier = int(step_multiplier)
        self.goal_dist_thresh = int(step_multiplier)

        self.neighbors = [(0, 1*step_multiplier), (0, -1*step_multiplier), (1*step_multiplier, 0), (-1*step_multiplier, 0),
                          (1*step_multiplier, 1*step_multiplier), (1*step_multiplier, -1*step_multiplier),
                          (-1*step_multiplier, 1*step_multiplier), (-1*step_multiplier, -1*step_multiplier)]

        self.close_set = set()
        self.came_from = {}
        self.gscore = {self.start: 0}
        self.fscore = {self.start: self.heuristic(self.start, self.goal)}
        self.oheap = []
        heappush(self.oheap, (self.fscore[self.start], self.start))

        self.init_map()

        self.dynamic_no_flight_zones = {}
        rospy.Subscriber("/utm/dynamic_no_fly_zones", String, self.on_dynamic_no_fly_zones)

        rospy.wait_for_service('/utm_parser/get_snfz')
        self.get_dnfz_handle = rospy.ServiceProxy('/utm_parser/get_dnfz', get_dnfz)

        dnfz_string = self.get_dnfz_handle()
        self.string_to_dnfz(dnfz_string)

        self.safety_extra_time = 0
        self.safety_dist_to_dnfz = 0

    def string_to_dnfz(self, string):

        data = str(string.dnfz)
        try:
            all_json_objs = json.loads(data)
            for json_obj in all_json_objs:
                #print json_obj
                self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                if json_obj["geometry"] == "polygon":
                    self.make_polygon(json_obj)
        except ValueError as Err:
            print("[AStar]: Couldn't convert string from UTM server to json..", Err)
            print(data)
        #self.plot_dnfz()

    def on_dynamic_no_fly_zones(self, msg):
        '''
        Callback function
        Use "int_id" in the string as key in the dict of dnfz
        '''

        string_msg = str(msg.data)


        try:
            all_json_objs = json.loads(string_msg)
            for json_obj in all_json_objs:
                if json_obj["int_id"] in self.dynamic_no_flight_zones:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                else:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                    if json_obj["geometry"] == "polygon":
                        self.make_polygon(json_obj)
        except ValueError as Err:
            print("Collision Detector: Couldn't convert string from UTM server to json..", Err)

    def make_polygon(self, json_obj):
        coords = json_obj["coordinates"]
        coords = coords.split(" ")
        i = 0
        for a in coords:
            coords[i] = a.split(',')
            i += 1
        list_of_points = []
        for point in coords:
            temp_coord = Coordinate(lat=float(point[1]), lon=float(point[0]))
            list_of_points.append(geometry.Point(temp_coord.easting, temp_coord.northing))
        # Taken from:
        # https://stackoverflow.com/questions/30457089/how-to-create-a-polygon-given-its-point-vertices
        dnfz_polygon = geometry.Polygon([[point.x, point.y] for point in list_of_points])
        self.dynamic_no_flight_zones[json_obj["int_id"]]["polygon"] = dnfz_polygon

    def plot_dnfz(self):

        np.random.seed(19680801)

        fig, ax = plt.subplots()

        patches = []

        """
        np.random.seed(19680801)
        print "Dynamic, no flight zones: ", self.dynamic_no_flight_zones
        #fig = plt.figure(1, figsize=(5,5), dpi=90)
        #big = plt.figure(1, figsize=(5,5), dpi=90)
        fig, ax = plt.subplots()
    
        patches = []
        """
        for int_id, dnfz in self.dynamic_no_flight_zones.items():
            coord = dnfz["coordinates"]

            if dnfz["geometry"] == "polygon":
                coord = coord.split(" ")
                i = 0
                for a in coords:
                    coord[i] = a.split(',')
                    i += 1
                list_of_points = []
                for i in coord:
                    one_coord = Coordinate(lat=i[1], lon=i[0])
                    list_of_points.append([one_coord.easting, one_coord.northing])
                poly = Polygon(list_of_points, True)
                patches.append(poly)
            else:
                coord = coord.split(",")
                one_coord = Coordinate(lat=coord[1], lon=coord[0])
                circle = Circle((one_coord.easting, one_coord.northing), coord[2])
                patches.append(circle)

        colors = 100 * np.random.rand(len(patches))
        p = PatchCollection(patches, alpha=0.4)
        p.set_array(np.array(colors))
        ax.add_collection(p)

        plt.ylim(1141400, 1142400)
        plt.xlim(332400, 333400)
        fig.colorbar(p, ax=ax)

        plt.show()



    def heuristic(self, a, b):
        return sqrt(((b[0] - a[0]) ** 2) + ((b[1] - a[1]) ** 2))

    def init_map(self):
        c2 = len(self.map.snfz_map) - 1
        for line in reversed(self.map.snfz_map):
            c1 = len(line.row) - 1
            for i in reversed(line.row):
                if i != 0:
                    self.map_image[c2][c1] = 255
                c1 -= 1
            c2 -= 1

    def clear_dicts_and_lists(self, dynamic=False):
        if not dynamic:
            self.close_set = set()
            self.came_from = {}
            self.gscore = {self.start[0:2]: 0}
            self.fscore = {self.start[0:2]: self.heuristic(self.start, self.goal)}
            self.oheap = []
            heappush(self.oheap, (self.fscore[self.start[0:2]], self.start[0:2]))
        else:
            self.close_set = set()
            self.came_from = {}
            self.gscore = {self.start: 0}
            self.fscore = {self.start: self.heuristic(self.start, self.goal)}
            self.oheap = []
            heappush(self.oheap, (self.fscore[self.start], self.start))

    def set_start_and_goal(self, start, goal, start_time=0):
        self.start = (start.easting, start.northing, start_time)
        self.goal = (goal.easting, goal.northing, 0)

    def set_step_multiplier(self, s):
        self.step_multiplier = int(s)
        self.neighbors = [(0, 1 * s), (0, -1 * s), (1 * s, 0), (-1 * s, 0),
                          (1 * s, 1 * s), (1 * s, -1 * s), (-1 * s, 1 * s), (-1 * s, -1 * s)]

    def compute_astar(self, dynamic=False, ground_speed=5):
        self.clear_dicts_and_lists(dynamic)
        #print(self.map_image.shape)
        #print(self.heuristic(self.start,self.goal))
        while self.oheap:
            if dynamic:
                current = heappop(self.oheap)[1]
            else:
                current = heappop(self.oheap)[1][0:2]

            #print "[ASTAR]: Current possition: ", current

            if self.goal_dist_thresh >= self.heuristic(current, self.goal):
                data = []
                while current in self.came_from:
                    coord = Coordinate(easting=current[0], northing=current[1])
                    data.append(coord)
                    print(current)
                    current = self.came_from[current]
                return data

            self.close_set.add(current[0:2])
            for i, j in self.neighbors:
                if dynamic:
                    new_time = current[2] + (self.heuristic(current, (current[0]+i, current[1]+j))/ground_speed)
                    neighbor = (current[0]+i, current[1]+j, new_time)
                else:
                    neighbor = (current[0]+i, current[1]+j)

                #print "[ASTAR]: Neighbor: ", neighbor

                tentative_g_score = self.gscore[current] + self.heuristic(current, neighbor)

                neighbor_transform = (int((neighbor[0] - self.lower_left.easting) / self.map_res),
                                      int((neighbor[1] - self.lower_left.northing) / self.map_res))

                if neighbor_transform[0] < self.map_width and neighbor_transform[1] < self.map_height:
                    if self.map_image[neighbor_transform[1]][neighbor_transform[0]] != 0:
                        #print "[ASTAR]: Point is in SNFZ"
                        continue

                if dynamic and self.is_collision_with_dnfz(neighbor):
                    #print "[ASTAR]: Point is in DNFZ"
                    continue

                if neighbor[0:2] in self.close_set and tentative_g_score >= self.gscore.get(neighbor, 0):
                    #print "[ASTAR]: Point is already checked, and shorter path exist"
                    continue

                if tentative_g_score < self.gscore.get(neighbor, 0) or neighbor not in [i[1] for i in self.oheap]:
                    #print "[ASTAR]: Adding valid point"
                    self.came_from[neighbor] = current
                    self.gscore[neighbor] = tentative_g_score
                    self.fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heappush(self.oheap, (self.fscore[neighbor], neighbor))
        #print "Astar closed set:", len(self.close_set)
        return False

    def is_collision_with_dnfz(self, neighbor):
        for int_id, dnfz in self.dynamic_no_flight_zones.items():
            if dnfz["geometry"] == "circle":
                coord = dnfz["coordinates"]
                coord = coord.split(',')
                idunno = Coordinate(lon=float(coord[0]), lat=float(coord[1]))
                dist = self.heuristic(neighbor, (idunno.easting, idunno.northing))
                if (int(dnfz["valid_from_epoch"]) - self.safety_extra_time) < neighbor[2] < (
                        int(dnfz["valid_to_epoch"]) + self.safety_extra_time):
                    if dist <= (coord[2] + self.safety_dist_to_dnfz):
                        return True
            elif dnfz["geometry"] == "polygon":
                dist = dnfz["polygon"].distance(geometry.Point(neighbor[0], neighbor[1]))
                if (int(dnfz["valid_from_epoch"]) - self.safety_extra_time) < neighbor[2] < (
                        int(dnfz["valid_to_epoch"]) + self.safety_extra_time):
                    if dist <= self.safety_dist_to_dnfz:
                        return True
        return False


if __name__ == "__main__":
    start = Coordinate(lat=55.481202, lon=10.344599)
    goal = Coordinate(lat=55.479860, lon=10.340543)

    print(sqrt((start.easting-goal.easting)**2 + (start.northing-goal.northing)**2))

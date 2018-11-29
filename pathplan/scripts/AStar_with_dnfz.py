#!/usr/bin/env python
# Based on:
# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python (2.7)
# Please give credit if used

from heapq import *
from math import sqrt
from coordinate import Coordinate
import numpy as np
import rospy
import time
import json
import string
from shapely import geometry
from collections import namedtuple
#from utm_parser.srv import *
#from utm_parser.msg import *

TimeCoord = namedtuple('TimeCoord', ['easting', 'northing', 'time'])
TimeCoord.__new__.__defaults__ = (None,) * len(TimeCoord._fields)


class AStar:
    def __init__(self, coord_start, coord_goal, static_map, map_padding, step_multiplier=1):
        temp_start = Coordinate(GPS_data=coord_start)
        temp_goal = Coordinate(GPS_data=coord_goal)
        self.start = TimeCoord(easting=temp_start.easting, northing=temp_start.northing)
        self.goal = TimeCoord(easting=temp_goal.easting, northing=temp_goal.northing)
        self.map = static_map
        self.map_res = static_map.resolution
        self.map_padding = map_padding
        self.map_image = np.zeros((static_map.map_width, static_map.map_height, 1), np.uint8)
        self.lower_left = Coordinate(easting=self.start.easting - map_padding,
                                     northing=self.start.northing - map_padding)
        self.upper_right = Coordinate(easting=self.start.easting + map_padding,
                                      northing=self.start.northing + map_padding)
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
        rospy.Substriber("/utm/dynamic_no_fly_zones", string, self.on_dynamic_no_fly_zones)

        self.safety_extra_time = 10
        self.safety_dist_to_dnfz = 10

    def on_dynamic_no_fly_zones(self, msg):
        '''
        Callback function
        Use "int_id" in the string as key in the dict of dnfz
        '''
        try:
            all_json_objs = json.loads(msg)
            for json_obj in all_json_objs:
                if json_obj["int_id"] in self.dynamic_no_flight_zones:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                else:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                    if json_obj["geometry"] == "polygon":
                        self.make_polygon(json_obj)
        except ValueError:
            print("Collision Detector: Couldn't convert string from UTM server to json..")

    def make_polygon(self, json_obj):
        coords = json_obj["coordinates"]
        coords = coords.split(" ")
        i = 0
        for a in coords:
            coords[i] = a.split(',')
            i += 1
        list_of_points = []
        for point in coords:
            temp_coord = Coordinate(lat=point[1], lon=point[0])
            list_of_points.append(geometry.Point(temp_coord.easting, temp_coord.northing))
        # Taken from:
        # https://stackoverflow.com/questions/30457089/how-to-create-a-polygon-given-its-point-vertices
        dnfz_polygon = geometry.Polygon([[point.x, point.y] for point in list_of_points])
        self.dynamic_no_flight_zones[json_obj["int_id"]]["polygon"] = dnfz_polygon

    def heuristic(self, a, b):
        return sqrt(((b.easting - a.easting) ** 2) + ((b.northing - a.northing) ** 2))

    def init_map(self):
        c2 = len(self.map.snfz_map) - 1
        for line in reversed(self.map.snfz_map):
            c1 = len(line.row) - 1
            for i in reversed(line.row):
                if i != 0:
                    self.map_image[c2][c1] = 255
                c1 -= 1
            c2 -= 1

    def clear_dicts_and_lists(self):
        self.close_set = set()
        self.came_from = {}
        start_set = (self.start.easting, self.start.northing)
        self.gscore = {start_set: 0}
        self.fscore = {start_set: self.heuristic(self.start, self.goal)}
        self.oheap = []
        heappush(self.oheap, (self.fscore[start_set], start_set))

    def set_start_and_goal(self, start, goal, start_time=0):
        temp_start = Coordinate(GPS_data=start)
        temp_goal = Coordinate(GPS_data=goal)
        self.start = TimeCoord(easting=temp_start.easting, northing=temp_start.northing, time=start_time)
        self.goal = TimeCoord(easting=temp_goal.easting, northing=temp_goal.northing)

    def set_step_multiplier(self, s):
        self.step_multiplier = int(s)
        self.neighbors = [(0, 1 * s), (0, -1 * s), (1 * s, 0), (-1 * s, 0),
                          (1 * s, 1 * s), (1 * s, -1 * s), (-1 * s, 1 * s), (-1 * s, -1 * s)]

    def compute_astar(self, dynamic=False, ground_speed=5):
        self.clear_dicts_and_lists()
        while self.oheap:
            current = heappop(self.oheap)[1]

            if self.goal_dist_thresh >= self.heuristic(current, self.goal):
                data = []
                while current in self.came_from:
                    coord = Coordinate(easting=current.easting, northing=current.northing)
                    data.append(coord)
                    current = self.came_from[current]
                return data

            current_set = (current.easting, current.northing)
            self.close_set.add(current_set)
            for i, j in self.neighbors:
                if dynamic:
                    new_time = current.time + (self.heuristic(current, TimeCoord(easting=current.easting+i,
                                                                                 northing=current.northing+j))/ground_speed)
                    neighbor = TimeCoord(easting=current.easting + i, northing=current.northing + j, time=new_time)
                else:
                    neighbor = TimeCoord(easting=current.easting + i, northing=current.northing + j)

                tentative_g_score = self.gscore[current_set] + self.heuristic(current, neighbor)

                neighbor_transform = (int((neighbor.easting - self.lower_left.easting) / self.map_res),
                                      int((neighbor.northing - self.lower_left.northing) / self.map_res))

                if self.map_image[neighbor_transform[1]][neighbor_transform[0]] != 0:
                    continue

                if dynamic and self.is_collision_with_dnfz(neighbor):
                    continue

                neighbor_set = (neighbor.easting, neighbor.northing)
                if neighbor_set in self.close_set and tentative_g_score >= self.gscore.get(neighbor_set, 0):
                    continue

                if tentative_g_score < self.gscore.get(neighbor_set, 0) or neighbor_set not in [i[1] for i in self.oheap]:
                    self.came_from[neighbor_set] = current_set
                    self.gscore[neighbor_set] = tentative_g_score
                    self.fscore[neighbor_set] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heappush(self.oheap, (self.fscore[neighbor_set], neighbor_set))
        return False

    def compute_static_astar(self):
        self.clear_dicts_and_lists()
        while self.oheap:
            current = heappop(self.oheap)[1]

            if current == self.goal or self.goal_dist_thresh >= self.heuristic(current, self.goal):
                data = []
                while current in self.came_from:
                    data.append(current)
                    current = self.came_from[current]
                return data

            self.close_set.add(current)
            for i, j in self.neighbors:
                neighbor = TimeCoord(easting=current.easting+i, northing=current.northing+j)
                tentative_g_score = self.gscore[current] + self.heuristic(current, neighbor)

                neighbor_transform = (int((neighbor.easting - self.lower_left.easting) / self.map_res),
                                      int((neighbor.northing - self.lower_left.northing) / self.map_res))
                if self.map_image[neighbor_transform[1]][neighbor_transform[0]] != 0:
                    continue

                if neighbor in self.close_set and tentative_g_score >= self.gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < self.gscore.get(neighbor, 0) or neighbor not in [i[1] for i in self.oheap]:
                    self.came_from[neighbor] = current
                    self.gscore[neighbor] = tentative_g_score
                    self.fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heappush(self.oheap, (self.fscore[neighbor], neighbor))
        return False

    def compute_dynamic_astar(self, ground_speed=5):
        self.clear_dicts_and_lists()

        while self.oheap:
            current = heappop(self.oheap)[1]

            if (current.easting == self.goal.easting and current.northing == self.goal.northing) or \
                    self.goal_dist_thresh >= self.heuristic(current, self.goal):
                data = []
                while current in self.came_from:
                    coord = Coordinate(easting=current.easting, northing=current.northing)
                    data.append(coord)
                    current = self.came_from[current]
                return data

            self.close_set.add(current)
            for i, j in self.neighbors:
                new_time = current.time + (self.heuristic(current, TimeCoord(easting=current.easting+i,
                                                                             northing=current.northing+j))/ground_speed)
                neighbor = TimeCoord(easting=current.easting+i, northing=current.northing+j, time=new_time)
                tentative_g_score = self.gscore[current] + self.heuristic(current, neighbor)

                neighbor_transform = (int((neighbor.easting - self.lower_left.easting) / self.map_res),
                                      int((neighbor.northing - self.lower_left.northing) / self.map_res))

                if self.map_image[neighbor_transform[1]][neighbor_transform[0]] != 0:
                    continue

                if self.is_collision_with_dnfz(neighbor):
                    continue

                if neighbor in self.close_set and tentative_g_score >= self.gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < self.gscore.get(neighbor, 0) or neighbor not in [i[1] for i in self.oheap]:
                    self.came_from[neighbor] = current
                    self.gscore[neighbor] = tentative_g_score
                    self.fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heappush(self.oheap, (self.fscore[neighbor], neighbor))
        return False

    def is_collision_with_dnfz(self, neighbor):
        for int_id, dnfz in self.dynamic_no_flight_zones.items():
            if dnfz["geometry"] == "circle":
                coord = dnfz["coordinates"]
                coord = coord.split(',')
                dist = self.heuristic(neighbor, Coordinate(lon=coord[0], lat=coord[1]))
                if (dnfz["valid_from_epoch"] - self.safety_extra_time) < neighbor.time < (
                        dnfz["valid_to_epoch"] + self.safety_extra_time):
                    if dist <= (coord[2] + self.safety_dist_to_dnfz):
                        return True
            elif dnfz["geometry"] == "polygon":
                dist = dnfz["polygon"].distance(geometry.Point(neighbor.easting, neighbor.northing))
                if (dnfz["valid_from_epoch"] - self.safety_extra_time) < neighbor.time < (
                        dnfz["valid_to_epoch"] + self.safety_extra_time):
                    if dist <= self.safety_dist_to_dnfz:
                        return True
        return False


if __name__ == "__main__":
    start_pos = Coordinate(lat=55.481202, lon=10.344599)
    goal_pos = Coordinate(lat=55.479860, lon=10.340543)

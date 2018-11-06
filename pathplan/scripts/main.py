#!/usr/bin/python

import numpy as np
from coordinate import Coordinate
from AStar_1 import astar
from AStar_2 import astar as newAstar
from exportkml import kmlclass
from math import sqrt
import time
from Path_simplifier import PathSimplifier
#from beginner_tutorials.srv import *
#import rospy
#from gcs.msg import GPS

# Global variables
# Default position at HCA Airport: lat=55.470415, lon=10.329449
global_bottom_left = Coordinate(lat=55.470415, lon=10.329449)
global_top_right = Coordinate(lat=55.470415, lon=10.329449)
global_map_width = 0
global_map_height = 0
path = []

# Global constants
map_padding = 100               # padding on top, bottom, left and right of map


# Helper functions
def set_global_coordinates(start, goal):
    global global_top_right, global_bottom_left
    if start.northing < goal.northing:
        if start.easting < goal.easting:
            global_bottom_left.northing = start.northing - map_padding
            global_bottom_left.easting = start.easting - map_padding
            global_top_right.northing = goal.northing + map_padding
            global_top_right.easting = goal.easting + map_padding
        else:
            global_bottom_left.northing = start.northing - map_padding
            global_bottom_left.easting = goal.easting - map_padding
            global_top_right.northing = goal.northing + map_padding
            global_top_right.easting = start.easting + map_padding
    else:
        if start.easting < goal.easting:
            global_bottom_left.northing = goal.northing - map_padding
            global_bottom_left.easting = start.easting - map_padding
            global_top_right.northing = start.northing + map_padding
            global_top_right.easting = goal.easting + map_padding
        else:
            global_bottom_left.northing = goal.northing - map_padding
            global_bottom_left.easting = goal.easting - map_padding
            global_top_right.northing = start.northing + map_padding
            global_top_right.easting = start.easting + map_padding

    global_bottom_left.update_geo_coordinates()
    global_top_right.update_geo_coordinates()

    global global_map_width, global_map_height
    global_map_width = int(global_top_right.easting - global_bottom_left.easting)
    global_map_height = int(global_top_right.northing - global_bottom_left.northing)


def make_map_zeros():
    return np.zeros((global_map_height, global_map_width), dtype=np.int)


def compute_rel_pos_and_map(start, goal):
    set_global_coordinates(start, goal)

    map_zeros = make_map_zeros()
    rel_start_pos = (int(start.northing - global_bottom_left.northing), int(start.easting - global_bottom_left.easting))
    rel_goal_pos = (int(goal.northing - global_bottom_left.northing), int(goal.easting - global_bottom_left.easting))

    # print("start (int): ", rel_start_pos, "goal (int): ", rel_goal_pos)

    return map_zeros, rel_start_pos, rel_goal_pos


def convert_from_rel_pos_to_real_coord(path, start, goal):
    path_real = []
    path_real.append(start)
    for j in reversed(path):
        path_real.append(Coordinate(northing=(j[0] + global_bottom_left.northing),
                                    easting=(j[1] + global_bottom_left.easting)))
    path_real.append(goal)
    return path_real


def compute_path_with_astar(start, goal):
    map_zeros, rel_start_pos, rel_goal_pos = compute_rel_pos_and_map(start, goal)

    path_relative = astar(map_zeros, rel_start_pos, rel_goal_pos)

    path_real = convert_from_rel_pos_to_real_coord(path_relative, start, goal)

    return path_real


if __name__ == "__main__":

    # Small runway
    #start_pos = Coordinate(lat=55.481202, lon=10.344599)
    #goal_pos = Coordinate(lat=55.479860, lon=10.340543)

    # All of runway
    start_pos = Coordinate(lat=55.471520, lon=10.315290)
    goal_pos = Coordinate(lat=55.481202, lon=10.344599)

    # Old pathplanner with map:
    #path = compute_path_with_astar(start_pos, goal_pos)

    print("Distance: ",
          sqrt((start_pos.easting - goal_pos.easting) ** 2 + (start_pos.northing - goal_pos.northing) ** 2))

    # New without map:
    t0 = time.time()
    path_reversed = newAstar(start_pos, goal_pos, step_multiplier=8)
    t1 = time.time()

    print("Found a path in %s seconds." % (t1 - t0))

    path = []
    for j in reversed(path_reversed):
        path.append(j)
    path.pop(-1)
    path.append(goal_pos)

    '''
    print("hem: ", path[0].hemisphere,
          "zone: ", path[0].zone,
          "letter: ", path[0].letter,
          "east: ", path[0].easting,
          "north: ", path[0].northing,)
    '''

    #for i in path:
    #    print("Northing: ", i.northing, "Easting: ", i.easting)

    print("waypoints: ", len(path))

    ps = PathSimplifier(path, step_size=16)
    ps.delete_with_step_size_safe(threshold=8)
    path = ps.get_simple_path()

    print("waypoints: ", len(path))

    # width: defines the line width, use e.g. 0.1 - 1.0
    kml = kmlclass()
    kml.begin('new_astar.kml', 'Example', 'Example on the use of kmlclass', 0.1)
    # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
    # altitude: use 'absolute' or 'relativeToGround'
    kml.trksegbegin('', '', 'red', 'absolute')
    for i in path:
        kml.trkpt(i.lat, i.lon, 0.0)
    kml.trksegend()
    kml.end()


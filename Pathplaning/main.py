#!/usr/bin/python

import numpy as np
from coordinate import coordinate
from utm import utmconv
from AStar_1 import astar
from exportkml import kmlclass

# Global variables
global_bottom_left = coordinate(0, 0)
global_top_right = coordinate(0, 0)
global_map_width = 0
global_map_height = 0
global_map = 0

path = []

# Global constants
geodetic_multiplier = 10000     # 1,000 ≈ 64m, 10,000 ≈ 6m, #100,000 ≈ 1m
map_padding = 10                # padding on top, bottom, left and right of map

# Helper functions
def set_global_coordinates(start, goal):
    global global_top_right, global_bottom_left
    if start.lat < goal.lat:
        if start.lon < goal.lon:
            global_bottom_left.lat = start.lat - map_padding
            global_bottom_left.lon = start.lon - map_padding
            global_top_right.lat = goal.lat + map_padding
            global_top_right.lon = goal.lon + map_padding
        else:
            global_bottom_left.lat = start.lat - map_padding
            global_bottom_left.lon = goal.lon - map_padding
            global_top_right.lat = goal.lat + map_padding
            global_top_right.lon = start.lon + map_padding
    else:
        if start.lon < goal.lon:
            global_bottom_left.lat = goal.lat - map_padding
            global_bottom_left.lon = start.lon - map_padding
            global_top_right.lat = start.lat + map_padding
            global_top_right.lon = goal.lon + map_padding
        else:
            global_bottom_left.lat = goal.lat - map_padding
            global_bottom_left.lon = goal.lon - map_padding
            global_top_right.lat = start.lat + map_padding
            global_top_right.lon = start.lon + map_padding
    global global_map_width, global_map_height
    global_map_width = int(global_top_right.lon - global_bottom_left.lon)
    global_map_height = int(global_top_right.lat - global_bottom_left.lat)

def multiply_geodetic(coord):
    coord.lat *= geodetic_multiplier
    coord.lon *= geodetic_multiplier
    return coord

def divide_geodetic(coord):
    coord.lat /= geodetic_multiplier
    coord.lon /= geodetic_multiplier
    return coord

def make_map_zeros():
    return np.zeros((global_map_height, global_map_width), dtype=np.int)

def convert_to_make_path(start, goal):
    start = multiply_geodetic(start)
    goal = multiply_geodetic(goal)

    set_global_coordinates(start, goal)

    map_zeros = make_map_zeros()
    rel_start_pos = (int(start.lat - global_bottom_left.lat), int(start.lon - global_bottom_left.lon))
    rel_goal_pos = (int(goal.lat - global_bottom_left.lat), int(goal.lon - global_bottom_left.lon))

    return map_zeros, rel_start_pos, rel_goal_pos

def convert_back_to_geodetic(path, start, goal):
    path_geo = []
    start = divide_geodetic(start)
    goal = divide_geodetic(goal)
    path_geo.append(start)
    for i in reversed(path):
        path_geo.append(coordinate((i[0] + global_bottom_left.lat)/geodetic_multiplier,
                                   (i[1] + global_bottom_left.lon)/geodetic_multiplier))
    path_geo.append(goal)
    return path_geo

def compute_path_with_astar(start, goal):
    pass

if __name__ == '__main__':
    start_pos = coordinate(55.470273, 10.329738)
    goal_pos = coordinate(55.481152, 10.344523)

    map_zeros, rel_start_pos, rel_goal_pos = convert_to_make_path(start_pos, goal_pos)

    path = astar(map_zeros, rel_start_pos, rel_goal_pos)

    path_geodetic = convert_back_to_geodetic(path, start_pos, goal_pos)

    print("hem: ", path_geodetic[0].hemisphere,
          "zone: ", path_geodetic[0].zone,
          "letter: ", path_geodetic[0].letter,
          "east: ", path_geodetic[0].easting,
          "north: ", path_geodetic[0].northing,)

    #for i in path_geodetic:
    #    print("lat: ", i.lat, "lon: ", i.lon)

    # width: defines the line width, use e.g. 0.1 - 1.0
    kml = kmlclass()
    kml.begin('testfile.kml', 'Example', 'Example on the use of kmlclass', 0.1)
    # color: use 'red' or 'green' or 'blue' or 'cyan' or 'yellow' or 'grey'
    # altitude: use 'absolute' or 'relativeToGround'
    kml.trksegbegin('', '', 'red', 'absolute')
    for i in path_geodetic:
        kml.trkpt(i.lat, i.lon, 0.0)
    kml.trksegend()
    kml.end()


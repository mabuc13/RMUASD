#!/usr/bin/env python
# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python (2.7)
# Please give credit if used

import numpy
from heapq import *
from math import sqrt
from coordinate import Coordinate
import cv2
import numpy as np
import rospy
from utm_parser.srv import *
from utm_parser.msg import *

def heuristic(a, b):
    return sqrt(((b[0] - a[0]) ** 2) + ((b[1] - a[1]) ** 2))


def static_collision_detector(a):
    return False


def astar(coord_start, coord_goal, step_multiplier=1, dynamic=False):
    start = (int(coord_start.easting), int(coord_start.northing))
    goal = (int(coord_goal.easting), int(coord_goal.northing))
    
    resolution = 2500

    lower_left = Coordinate(easting=coord_start.easting-resolution, northing=coord_start.northing-resolution)
    upper_right = Coordinate(easting=coord_start.easting+resolution, northing=coord_start.northing+resolution)


    rospy.wait_for_service('/utm_parser/get_snfz')
    get_snfz_handle = rospy.ServiceProxy('/utm_parser/get_snfz', get_snfz)
    map = get_snfz_handle(lower_left.GPS_data, upper_right.GPS_data)


    map_res = map.resolution
    print "Map resoluton: ", map_res
    map_image = np.zeros((map.map_width, map.map_height, 1), np.uint8)

    c2 = len(map.snfz_map) -1
    for line in reversed(map.snfz_map):
        c1 = len(line.row) -1 
        for i in reversed(line.row):
            if i != 0:
                map_image[c2][c1] = 255 #map.map_width -
                #print "Found NFZ at: ", map.map_width - c2, c1
                #map_image[map.map_width - c2][c1][1] = 255
                #map_image[map.map_width - c2][c1][2] = 255
            c1 -= 1
        c2 -= 1


    goal_dist_thresh = step_multiplier
    neighbors = [(0, 1*step_multiplier), (0, -1*step_multiplier), (1*step_multiplier, 0), (-1*step_multiplier, 0),
                 (1*step_multiplier, 1*step_multiplier), (1*step_multiplier, -1*step_multiplier),
                 (-1*step_multiplier, 1*step_multiplier), (-1*step_multiplier, -1*step_multiplier)]

    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []

    heappush(oheap, (fscore[start], start))

    while oheap:

        current = heappop(oheap)[1]

        if current == goal or goal_dist_thresh >= heuristic(current, goal):
            data = []
            while current in came_from:
                coord = Coordinate(easting=current[0], northing=current[1])
                data.append(coord)
                neighbor_transform = (int((coord.easting - lower_left.easting) / map_res), int((coord.northing - lower_left.northing) / map_res))
                map_image[neighbor_transform[1]][neighbor_transform[0]] = 150
                current = came_from[current]
            map_image = cv2.flip(map_image, 0)
            """
            cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
            cv2.imshow('Map', map_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            """
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # Collision detection:
            neighbor_transform = (int((neighbor[0]-lower_left.easting)/map_res), int((neighbor[1]-lower_left.northing)/map_res))
            #print neighbor_transform
            if map_image[neighbor_transform[1]][neighbor_transform[0]] != 0:
                # No-fly-zone
                #print map_image[neighbor_transform[0]][neighbor_transform[1]]
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heappush(oheap, (fscore[neighbor], neighbor))

    return False


if __name__ == '__main__':
    # Small runway
    start_pos = Coordinate(lat=55.481202, lon=10.344599)
    goal_pos = Coordinate(lat=55.479860, lon=10.340543)

    print("Distance: ", heuristic((int(start_pos.easting), int(start_pos.northing)), (int(goal_pos.easting), int(goal_pos.northing))))

    path = astar(start_pos, goal_pos, step_multiplier=5)

    print(path)

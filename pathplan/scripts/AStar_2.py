#!/usr/bin/env python
# Author: Christian Careaga (christian.careaga7@gmail.com)
# A* Pathfinding in Python (2.7)
# Please give credit if used

import numpy
from heapq import *
from math import sqrt
from coordinate import Coordinate


def heuristic(a, b):
    return sqrt(((b[0] - a[0]) ** 2) + ((b[1] - a[1]) ** 2))


def static_collision_detector(a):
    return False


def astar(coord_start, coord_goal, step_multiplier=1, dynamic=False):
    start = (int(coord_start.easting), int(coord_start.northing))
    goal = (int(coord_goal.easting), int(coord_goal.northing))

    goal_dist_thresh = step_multiplier/2
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
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            # Collision detection:
            if static_collision_detector(neighbor):
                # No-fly-zone
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

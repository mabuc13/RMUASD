#!/usr/bin/python

import numpy as np
from numpy import *  # Gradient
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
from coordinate import coordinate

padding = 6    # Padding in top, bottum, left and right of start and goal coordinates
delta = 1.0  # Strength of attraction
mu = 1.0  # Strength of repultion
dist_thresh = 0.02  #
objects = np.empty((0, 2), coordinate)
global_bottom_left = coordinate(0, 0)
global_top_right = coordinate(0, 0)
global_map_width = 0
global_map_height = 0

def set_global_coordinates(start, goal):
    if start.lat < goal.lat:
        if start.lon < goal.lon:
            global_bottom_left.lat = start.lat - padding
            global_bottom_left.lon = start.lon - padding
            global_top_right.lat = goal.lat + padding
            global_top_right.lon = goal.lon + padding
        else:
            global_bottom_left.lat = start.lat - padding
            global_bottom_left.lon = goal.lon - padding
            global_top_right.lat = goal.lat + padding
            global_top_right.lon = start.lon + padding
    else:
        if start.lon < goal.lon:
            global_bottom_left.lat = goal.lat - padding
            global_bottom_left.lon = start.lon - padding
            global_top_right.lat = start.lat + padding
            global_top_right.lon = goal.lon + padding
        else:
            global_bottom_left.lat = goal.lat - padding
            global_bottom_left.lon = goal.lon - padding
            global_top_right.lat = start.lat + padding
            global_top_right.lon = start.lon + padding

def dist(start, goal):
    return sqrt((start.lat - goal.lat) ** 2 + (start.lon - goal.lon) ** 2)

def dist_matrix(start, goal, obj):
    x, y = np.mgrid[global_bottom_left.lon:global_top_right.lon, global_bottom_left.lat:global_top_right.lat]
    dist = sqrt((obj.lon - x) ** 2 + (obj.lat - y) ** 2)
    return dist

def dist_gradient(start, goal, obj):
    x, y = np.mgrid[global_bottom_left.lon:global_top_right.lon, global_bottom_left.lat:global_top_right.lat]
    D = -(obj.lon - x) ** 2 - (obj.lat - y) ** 2
    eLat, eLon = gradient(D)
    return eLat, eLon

def obj_gradient(start, goal, obj):
    x, y = np.mgrid[global_bottom_left.lon:global_top_right.lon, global_bottom_left.lat:global_top_right.lat]
    D = -1 / (sqrt((y - obj.lat) ** 2 + (x - obj.lon) ** 2))
    eLat, eLon = gradient(D)
    return eLat, eLon

def U_att(start, goal):
    U_lat, U_lon = dist_gradient(start, goal, goal)
    return delta * (U_lat), delta * (U_lon),

def U_rep(start, goal):
    x, y = np.mgrid[global_bottom_left.lon:global_top_right.lon, global_bottom_left.lat:global_top_right.lat]#np.mgrid[start.lon:goal.lon + 1, start.lat:goal.lat + 1]
    U_lat, U_lon = np.zeros((2, global_map_width, global_map_height))
    # print 'U_lat: {}'.format(U_lat)

    for i in objects:
        distance = dist_matrix(start, goal, i)
        # print 'dist: {}'.format(distance)

        temp = mu * ((1 / dist_thresh) - (1 / distance) * (1 / (distance ** 2)))

        #print ("temp: ", temp)
        grad_lat, grad_lon = obj_gradient(start, goal, i)

        #print ("lat: ", grad_lat)
        #print ("lon: ", grad_lon)

        U_lat += grad_lat * temp
        U_lon += grad_lon * temp

    return U_lat, U_lon

def U_total(start, goal):
    U_a_lat, U_a_lon = U_att(start, goal)
    U_r_lat, U_r_lon = U_rep(start, goal)

    # print 'att_lat: {}'.format(U_a_lat)
    # print 'rep_lat: {}'.format(U_r_lat)

    return U_a_lat + U_r_lat, U_a_lon + U_r_lon

if __name__ == "__main__":
    obj_0 = coordinate(4, 6)
    obj_1 = coordinate(8, 2)
    obj_2 = coordinate(8, 12)
    obj_3 = coordinate(1, -3)

    objects = np.append(objects, obj_0)
    objects = np.append(objects, obj_1)
    objects = np.append(objects, obj_2)
    objects = np.append(objects, obj_3)

    start = coordinate(10, 10)
    goal = coordinate(1, 1)

    set_global_coordinates(start, goal)
    global_map_width = global_top_right.lon - global_bottom_left.lon
    global_map_height = global_top_right.lat - global_bottom_left.lat

    n = 10
    X, Y = np.mgrid[global_bottom_left.lon:global_top_right.lon, global_bottom_left.lat:global_top_right.lat]
    # np.mgrid[0:(global_map_width), 0:(global_map_height)]
    U, V = U_total(start, goal)

    circles = []

    fig = plt.figure()
    ax = fig.add_axes([0.025, 0.025, 0.95, 0.95])

    #plt.axes([0.025, 0.025, 0.95, 0.95])
    # plt.quiver(X, Y, U, V, R, alpha=.5)
    #circle1 = patches.Circle((8, 2), 0.3, fc='r', alpha=0.5, picker=True)
    #circle1 = patches.Circle((2, -3), 0.3, fc='r', alpha=0.5, picker=True)
    #circle = patches.Circle((-2, 3), 0.3, fc='b', alpha=0.5, picker=True)
    #circle3 = patches.Circle((-2, -3), 0.3, fc='b', alpha=0.5, picker=True)
    #circles.append(ax.add_patch(circle1))
    #circles.append(ax.add_patch(circle))
    #circles.append(ax.add_patch(circle2))
    #circles.append(ax.add_patch(circle3))
    plt.quiver(X, Y, U, V, edgecolor='k', facecolor='None', linewidth=.5)

    plt.xticks(())
    plt.yticks(())

    plt.show()
#!/usr/bin/python

#import numpy as np
from coordinate import Coordinate
#import rospy
from gcs.msg import GPS, DronePath
#from gcs.srv import *
import time
from math import sqrt


def distance_between_positions(start, goal):
    coord_start = Coordinate(lat=start.latitude, lon=start.latitude)
    coord_goal = Coordinate(lat=goal.latitude, lon=goal.latitude)
    return sqrt((coord_goal.northing - coord_start.northing)**2 + (coord_goal.easting - coord_start.easting)**2)


def eta_estimate(path, avr_speed):
    total_distance = 0.0
    for i in range(1, len(path)):
        total_distance += distance_between_positions(path[i-1], path[i])

    total_time_sec = total_distance * avr_speed
    current_time = time.time()
    eta_sec = current_time + total_time_sec

    return time.ctime(eta_sec)

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import requests
import json
import cv2
import numpy as np
import rospkg
import rospy
from gcs.msg import *

from node_monitor.msg import heartbeat
from std_msgs.msg import String

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from utm_parser.srv import *
from utm_parser.msg import *
#import sh
import csv
import simplekml
from termcolor import colored
from kml_reader import kml_no_fly_zones_parser
from time import gmtime, strftime
from utm import utmconv
import math
import string

def check_client():
    """
    tt = GPS()
    tt.latitude = 55.278524
    tt.longitude = 10.558190 #Coordinate in the middle of nowhere
    tt.altitude = 0
    """
    tt = GPS()
    tt.latitude = 55.472016
    tt.longitude = 10.415694 #Coordinate within the fake dnfz at modelflyvepladsen
    tt.altitude = 0




    rospy.wait_for_service('/utm_parser/is_coord_free')
    check_handle = rospy.ServiceProxy('/utm_parser/is_coord_free', is_coord_free)

    result = check_handle(tt)

    rospy.wait_for_service('/utm_parser/is_coord_free')
    rally_handle = rospy.ServiceProxy('/utm_parser/get_rally_points', get_rally_points)

    points = rally_handle()
    #print points
    #print "Rally points", points.data
    return result

if __name__ == "__main__":
    rospy.init_node('client_example')  # , anonymous=True)
    rospy.sleep(1)

    #utm_par = utm_parser()

    while True:
        rospy.sleep(5)
        is_free = check_client()
        print "Is free: ", is_free

    #utm_par.show_map(map_image) #If you iterate through the map array and makes it to a numpy array this can be used to show the map
    rospy.spin()

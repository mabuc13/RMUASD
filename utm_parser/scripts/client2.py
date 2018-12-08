#!/usr/bin/env python

import sys
import rospy

from gcs.msg import *
from utm_parser_node import *
import numpy as np
from utm_parser.srv import *
from utm_parser.msg import *
import cv2

def check_client():
    tt = GPS()
    tt.latitude = 55.472016
    tt.longitude = 10.415694 #Coordinate within the fake dnfz at modelflyvepladsen
    tt.altitude = 0

	
    rospy.wait_for_service('/utm_parser/is_coord_free')
    check_handle = rospy.ServiceProxy('/utm_parser/is_coord_free', is_coord_free)
    result = check_handle(tt)


    return result

if __name__ == "__main__":
    rospy.init_node('client_example')  # , anonymous=True)
    rospy.sleep(1)

    #utm_par = utm_parser()


    is_free = check_client()


    #utm_par.show_map(map_image) #If you iterate through the map array and makes it to a numpy array this can be used to show the map
    rospy.spin()

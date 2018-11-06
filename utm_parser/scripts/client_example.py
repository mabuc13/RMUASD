#!/usr/bin/env python

import sys
import rospy

from gcs.msg import *
from utm_parser_node import *
import numpy as np
from utm_parser.srv import *
from utm_parser.msg import *
import cv2

def snfz_client():
    ll = GPS()
    ll.latitude = 55.417960
    ll.longitude = 10.400059
    ll.altitude = 0



    ur = GPS()
    ur.latitude =  55.434906
    ur.longitude = 10.436443
    ur.altitude = 0.0
    #55.434839, 10.470696

    rospy.wait_for_service('/utm_parser/get_snfz')
    get_snfz_handle = rospy.ServiceProxy('/utm_parser/get_snfz', get_snfz)
    result = get_snfz_handle(ll, ur)
    print "Done getting SNFZ"

    return result

if __name__ == "__main__":
    rospy.init_node('client_example')  # , anonymous=True)
    rospy.sleep(1)

    #utm_par = utm_parser()


    map = snfz_client()
    map_image = np.zeros((map.map_width, map.map_height, 3), np.uint8)

    c2 = 0
    for line in map.snfz_map:
        c1 = 0
        for i in line.row:
            if i != 0:
                map_image[map.map_width-c2][c1][0] = 255
                map_image[map.map_width-c2][c1][1] = 255
                map_image[map.map_width-c2][c1][2] = 255
            c1 += 1
        c2 += 1

    cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
    cv2.imshow('Map', map_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    #utm_par.show_map(map_image) #If you iterate through the map array and makes it to a numpy array this can be used to show the map
    rospy.spin()

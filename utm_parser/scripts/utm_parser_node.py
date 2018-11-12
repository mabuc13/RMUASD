#!/usr/bin/env python
# -*- coding: utf-8 -*-
import signal
import sys
import rospy
import time
import struct

"""
Descriptors: MB = Mark Buch (mabuc13@student.sdu.dk)
2018-10-06 MB created file heavily inspired by example from Tobias Lundby @ SDU
"""

"""
Description:
This class will handle post and get to the UTM from the GCS. 
The idea is to have each request as a seperate method.

License: BSD 3-Clause
"""

import sys
import requests
import json
import cv2
import numpy as np
from gcs.msg import *

from utm_parser.srv import get_snfz
from utm_parser.msg import *
#import sh
import csv
import simplekml
from termcolor import colored
from kml_reader import kml_no_fly_zones_parser
from time import gmtime, strftime
from utm import utmconv
from math import acos, asin, sqrt, sin, cos, pi
# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

class utm_parser(object):
    def __init__(self):
        self.payload = {
            ''
        }
        self.kml = kml_no_fly_zones_parser(0)
        self.static_filename = ''
        self.geoditic_coords = []
        self.utm_coords = []
        self.coord_conv = utmconv()
        self.debug = 1
        self.utm_coords = []
        self.empty_map = []
        self.map_ll_reference = []
        self.map_res = 1
        self.ll_dummy = ['N', 32, 'U', 466703, 6162091]
        self.ur_dummy = ['N', 32, 'U', 469703, 6165091]
        self.map_length = 0
        self.map_width = 0
        self.map_height = 0
    #ROS STUFF
        self.get_snfz_service = rospy.Service("/utm_parser/get_snfz", get_snfz, self.get_snfz_handler, buff_size=10)

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")
    #Helper methods
    #GPS SYNTAX
    """
    fisk = GPS()
    fisk.longitude = 0
    fisk.latitude = 0
    fisk.altitude  = 9
    """

    def get_snfz_handler(self, req):

        map = self.get_static_nfz(req.coordinate_lower_left, req.coordinate_upper_right)
        #self.show_map(map)
        ros_map = []
        if self.debug:
            print "About to go from image to array for ROS"
        count_out = 0
        print len(map)
        print self.map_width, self.map_height
        for i in map:
            count_out += 1
            map_row = array1d()
            count_in = 0
            for j in i:
                count_in += 1
                temp = j[0]
                map_row.row.append(temp)

            ros_map.append(map_row)
            #print count_out
        if self.debug:
            "Done going from image to ros array"
        return ros_map, self.map_res, self.map_width, self.map_height

    def create_kml(self, file_name, data): #Formats special characters out of a string and saves it as a kml file with file_name

        if self.debug:
            print "Entering_create KML \n"
        f = open(file_name,"w+")
        line_array = data.split("\n")
        for i in line_array: #remove this for better performance as we anyway throw away the names
            i = i.replace('æ', 'ae')
            i = i.replace('ø', 'oo')
            i = i.replace('å', 'aa')

            i = i.replace('Æ', 'AE')
            i = i.replace('Ø', 'OO')
            i = i.replace('Å', 'AE')

            i = i.replace('é', 'e')
            i = i.replace('ö', 'oo')

            f.write(i)
            f.write("\n")

            #f.write("\n")

        f.close()
        if self.debug:
            print "Done creating KML \n"

    def get_dynamic_nfz(self):
        """
                input: An lower left coord and an upper right cord bounding the area checked for SNFZ
                output: TBD
                """
        if self.debug:
            print
            "Entering get_dynamic_NFZ \n"

        self.payload = {
            'data_type': 'dynamic_no_fly'
        }
        r = ''
        try:
            r = requests.get(url='https://droneid.dk/rmuasd/utm/data.php', params=self.payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            print
            colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            print
            colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:
            print
            colored('HTTP error', 'red')
            print
            colored(err, 'yellow')
            # sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print
            colored('Request error', 'red')
            print
            colored(err, 'yellow')
            sys.exit(1)
        else:
            print
            colored('Status code: %i' % r.status_code, 'yellow')
            print
            colored('Content type: %s' % r.headers['content-type'], 'yellow')

            data_dict = ''
            try:
                if self.debug:
                    print
                    "Entering the try section of get dynamic NFZ"
                data_dict = json.loads(r.text)


            except:
                print
                colored('Try part og get dynamic NFZ failed', 'red')
            else:
                if self.debug:
                    print
                    "Succesfully downloaded dynamic NFZ"
                #for entry in data_dict:
                right_now = time.time()
                try:

                    #print data_dict
                    print "Current time: ", time.time()
                    print "Trial and error: ", data_dict[0]['valid_from_epoch']
                    time_to_active = float(data_dict[0]['valid_from_epoch']) - right_now
                    print "Time to active: ", time_to_active
                    ending = float(data_dict[0]['valid_to_epoch'])
                    print "Time left: ", ending - right_now
                except Exception as e:
                    print e
                    rospy.logerr("Failed to retrieve DNFZ, maybe there is none")
                    rospy.logerr(e)
                else:
                    print "DNFZ data from the server: " , data_dict
                    return data_dict

    def get_static_nfz(self, coord_ll, coord_ur):
        """
        input: An lower left coord and an upper right cord bounding the area checked for SNFZ
        output: An numpy array where 255 means that the position corresponds to a no flight zone
        """
        if self.debug:
            print "Entering get_static_NFZ \n"

        self.payload = {
            'data_type': 'static_no_fly'
        }
        r = ''
        try:
            r = requests.get(url = 'https://droneid.dk/rmuasd/utm/data.php', params = self.payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            print colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:
            print colored('HTTP error', 'red')
            print colored(err, 'yellow')
            #sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print colored('Request error', 'red')
            print colored(err, 'yellow')
            sys.exit(1)
        else:
            print colored('Status code: %i' % r.status_code, 'yellow')
            print colored('Content type: %s' % r.headers['content-type'], 'yellow')

            data_dict = ''
            try:
                if self.debug:
                    print "Entering the try section of get static NFZ"
                string = r.content
                string = string[:-7]
                #filename = str(time.now())
                self.static_filename = strftime("%a, %d %b %Y %H:%M:%S +0000", gmtime())
                self.create_kml(self.static_filename, string)

                #self.kml.parse_file(self.static_filename)
                #self.geoditic_coords = self.kml.get_zones()

                #self.extract_coords(self.geoditic_coords)

            except:
                print colored('Error in parsing to ktml parser', 'red')
            else:
                if self.debug:
                    print "Entering last else part of get static NFZ"
                self.kml.parse_file(self.static_filename)
                self.geoditic_coords = self.kml.get_zones()

                self.utm_coords = self.extract_coords(self.geoditic_coords)
                #self.print_nested_list(self.utm_coords)
                utm_ll = self.coord_conv.geodetic_to_utm(coord_ll.latitude, coord_ll.longitude)
                utm_ur = self.coord_conv.geodetic_to_utm(coord_ur.latitude, coord_ur.longitude)
                if self.debug:
                    print "Utm ll, utm UR: ", utm_ll, utm_ur
                self.create_empty_map(utm_ll, utm_ur)
                return_map = self.snfz_into_empty_map(self.utm_coords, utm_ur, utm_ll)
                return return_map



    def extract_coords(self, g_coords):
        """
        This method gets rid of the names of the zones, and stores each coordinate set within a nested list
        each entry in this list corresponds to the coords defining one zone.
        The format of the retunred coordinates will be UTM

        :param g_coords: the data from the kml parser.
        :return: nested list with all coordinates in utm format

        """
        if self.debug:
            print "Entering extract coords"
        counter = 0
        try:

            if self.debug:
                print
                "Extracting and converting coordinates"

            for zone in g_coords:
                zone_transformed = []
                for element in zone['coordinates']:
                    utm_coord = self.coord_conv.geodetic_to_utm(element[0], element[1])
                    zone_transformed.append(utm_coord)
                self.utm_coords.append(zone_transformed)


        except Exception as e:
            print e
            rospy.logerr("Failed to extract coords from kml_parser")
            rospy.logerr(e)
        else:
            if self.debug:
                print "Sucessfully extracted coords"
            #print self.geoditic_coords

            return self.utm_coords



    def print_nested_list(self, nested_list):
        if self.debug:
            print "Entering print_nested_list"
        outer_cnt = 0
        for i in nested_list:
            print "Outer list element ", outer_cnt, " With coordinates: \n"

            for j in nested_list[outer_cnt]:
                print j, '\n'
            outer_cnt += 1


    def create_empty_map(self, ll_utm, ur_utm):
        delta_x = ur_utm[3] - ll_utm[3]
        delta_y = ur_utm[4] - ll_utm[4]
        if delta_x > delta_y:
            resolution = delta_x / 3000
        else:
            resolution = delta_y / 3000
        self.map_res = resolution

        width = int(delta_x/resolution)
        height = int(delta_y/resolution)
        self.map_width = width
        self.map_height = height
        self.empty_map = np.zeros((width, height, 3), np.uint8)
        if self.debug:
            print "Created empty map with height, width: ", height, width


    def snfz_into_empty_map(self, utm_coords, upper_right, down_left):
        if self.debug:
            print "Entered SNFZ into empty map"
        snfz_map = self.empty_map
        zone_counter = 0

        for i in utm_coords:
            current_zone = []
            for j in i:

                if j[3] < upper_right[3] and j[3] > down_left[3] and j[4] < upper_right[4] and j[4] > down_left[4]:
                    print "Found SNFZ within map!!"
                    index_width = int((j[3]- down_left[3])/self.map_res)
                    index_heigth = int((j[4]- down_left[4])/self.map_res)

                    #current_zone_width.append(index_width)
                    current_zone.append([index_width, index_heigth])
                    print "Zone counter: ", zone_counter
                    if snfz_map[index_width][index_heigth][0] == 0:
                        snfz_map[index_width][index_heigth][0] = zone_counter#snfz_map[index_width, index_heigth] = 1
                    if self.debug:
                        print "Found coordinate within map which is NFZ: ", j, "\n Which has index: ", index_width, index_heigth, "which is within zone. ", zone_counter
            if len(current_zone) != 0:
                print "About to make polygon where len is: ", len(current_zone)
                numpy_zone = np.array((current_zone),dtype=int)
                print numpy_zone
                #numpy_zone = numpy_zone.reshape((-1, 1, 2))
                dummy = np.array(([20, 20], [20, 100], [100, 100]), dtype=int)
                cv2.fillConvexPoly(snfz_map, numpy_zone, [255, 255, 255])
            zone_counter += 1

            #3 eastern 4 = northing

        if self.debug:
            print "Exited SNFZ into empty map, with map_res: ", self.map_res
        return snfz_map
        #self.show_map(snfz_map)
        #self.save_map(snfz_map)

    def save_map(self, map):
        f = open("most_recent_map.txt", "w+")
        #with open('most_recent_map.txt', 'w+') as f:
        for elem in map:

            for xs in elem:
                f.write(str(xs))
                f.write(" ")
            f.write('\n')


        f.close()

    def show_map(self, map):
        """map_image = np.zeros((self.map_length, self.map_length, 3), np.uint8)
        c1 = 0
        c2 = 0
        count3 = 0
        font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(img, 'OpenCV', (10, 500), font, 4, (255, 255, 255), 2, cv2.LINE_AA)
        for elem in map:
            c2 = 0
            for xs in elem:

                if xs != 0:

                    stri = str(xs)
                    cv2.putText(map_image, stri, (c1, c2), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
                    count3 += 1
                c2 += 1
            c1 += 1
        print count3
        """
        cv2.namedWindow('Map', cv2.WINDOW_NORMAL)
        cv2.imshow('Map', map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def print_test_coords(self):
        print "ll coord: ", self.coord_conv.utm_to_geodetic(self.ll_dummy[0], self.ll_dummy[1], self.ll_dummy[3], self.ll_dummy[4])
        print "UR coord: ", self.coord_conv.utm_to_geodetic(self.ur_dummy[0], self.ur_dummy[1], self.ur_dummy[3], self.ur_dummy[4])

    #def dnfz_timer_callback(self):
    #   current_dnfz = self.get_dynamic_nfz()



def main():
        rospy.init_node('utm_parser')#, anonymous=True)
        rospy.sleep(1)


        par = utm_parser()
        rospy.on_shutdown(par.shutdownHandler)
        #rospy.timer(rospy.Duration(5), par.dnfz_time_callback)
        #par.get_dynamic_nfz()
        #par.print_nested_list(par.geoditic_coords)
        #par.print_test_coords()

        #par.get_static_nfz()

        #par.print_zones()

        rospy.spin()

if __name__ == "__main__":
    main()
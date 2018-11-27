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
from node_monitor.msg import heartbeat
from std_msgs.msg import String

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
# Disable warning
from requests.packages.urllib3.exceptions import InsecureRequestWarning
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

class dict_init(dict):
    def __missing__(self, key):
        return 0

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
        self.utm_trafic_debug = 0
        self.debug = 0
        self.path_debug = 0
        self.utm_coords = []
        self.empty_map = []
        self.map_ll_reference = []
        self.map_res = 1
        self.ll_dummy = ['N', 32, 'U', 466703, 6162091]
        self.ur_dummy = ['N', 32, 'U', 469703, 6165091]
        self.map_length = 0
        self.map_width = 0
        self.map_height = 0
        self.path = []
        self.post_payload = {
            'uav_id': 3012,
            'uav_auth_key': '96ba4387cb37a2cbc5f05de53d5eab0c9583f1e102f8fe10ccab04c361234d6cd8cc47c0db4a46e569f03b61374745ebb433c84fac5f4bdfb8d89d2eb1d1ec0f',
            'uav_op_status': 3,
            'pos_cur_lat_dd': -1,
            'pos_cur_lng_dd': -1,
            'pos_cur_alt_m': -1,
            'pos_cur_hdg_deg': -1,
            'pos_cur_vel_mps': -1,
            'pos_cur_gps_timestamp': -1,
            'wp_next_lat_dd': -1,
            'wp_next_lng_dd': -1,
            'wp_next_alt_m': -1,
            'wp_next_hdg_deg': -1,
            'wp_next_vel_mps': -1,
            'wp_next_eta_epoch': -1,
            'uav_bat_soc': -1
        }
        self.last_info_pub = time.time()
        self.path_flag = False
        self.latest_dynamic_data = self.get_dynamic_nfz()
        self.published_first_dnfz = 0
        #ROS STUFF
        self.get_snfz_service = rospy.Service("/utm_parser/get_snfz", get_snfz, self.get_snfz_handler, buff_size=10)
        #self.post_drone_service = rospy.Service("/utm_parser/post_drone_info", post_drone_info, self.post_drone_info_handler, buff_size=10) ##SUBSCRIBE
        self.drone_info_sub = rospy.Subscriber("/drone_handler/DroneInfo", DroneInfo, self.post_drone_info_handler)
        self.path_sub = rospy.Subscriber("/gcs/forwardPath", DronePath, self.save_path) #Activity when new path is calculated

        self.dnfz_pub = rospy.Publisher('/utm/dynamic_no_fly_zones', String, queue_size=10)
        self.heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
        self.utm_drones_pub = rospy.Publisher('/utm/dronesList',UTMDroneList, queue_size=10)
        self.heart_msg = heartbeat()

        self.recent_drone = dict_init()
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
    def save_path(self, msg):
        self.path = msg.Path
        if len(self.path) > 0:
            self.path_flag = True
        else:
            self.path_flag = False
        if self.path_debug:
            print "Got path and stored it: ", self.path
            print self.path

    def get_snfz_handler(self, req):
        if self.debug:
            print "Processing get_snfz service with coordinates lower left: ", req.coordinate_lower_left
            print "and coordinate upper right: ", req.coordinate_upper_right
        map = self.get_static_nfz(req.coordinate_lower_left, req.coordinate_upper_right)
        #self.show_map(map)
        #self.show_map(map)
        ros_map = []
        if self.debug:
            print "About to go from image to array for ROS"
            print len(map)
            print self.map_width, self.map_height
        count_out = 0
        for i in map:
            count_out += 1
            map_row = array1d()
            count_in = 0
            for j in i:
                count_in += 1
                temp = j
                map_row.row.append(temp)

            ros_map.append(map_row)
            #print count_out
        if self.debug:
            "Done going from image to ros array"
        return ros_map, self.map_res, self.map_width, self.map_height

    def post_drone_info_handler(self, msg):
        dummy_payload = self.post_payload
        now = time.time()
        if now-self.last_info_pub >= 1:
            GPS_pos = msg.position
            self.post_payload['pos_cur_lat_dd'] = GPS_pos.latitude
            self.post_payload['pos_cur_lng_dd'] = GPS_pos.longitude
            self.post_payload['pos_cur_alt_m'] = msg.absolute_alt

            wp_geo = msg.next_waypoint
            """
            utm_pos = self.coord_conv.geodetic_to_utm(GPS_pos.latitude, GPS_pos.longitude)

            head_vec = [utm_wp[3]-utm_pos[3], utm_wp[4]-utm_pos[4]]
            if self.debug:
                print "Vector for heading: ", head_vec
            vec_degree = math.atan(head_vec[1]/head_vec[0])*180/math.pi #0 equals to east
            """

            self.post_payload['pos_cur_hdg_deg'] = msg.heading #Therefore adding 90 in a CCW manner will make 0 equals north
            self.post_payload['pos_cur_vel_mps'] = msg.ground_speed
            self.post_payload['pos_cur_gps_timestamp'] = -1
            #print "msgGPS: ", msg.GPS_timestamp, " msg.wp lon lat: ", msg.next_waypoint
            self.post_payload['wp_next_lat_dd'] = wp_geo.latitude
            self.post_payload['wp_next_lng_dd'] = wp_geo.longitude
            self.post_payload['wp_next_alt_m'] = wp_geo.altitude

            self.post_payload['uav_bat_soc'] = msg.battery_SOC

            if self.path_flag:

                if msg.mission_index+1 < len(self.path):
                    if self.debug:
                        print("Misssion ["+str(msg.mission_index+1)+"/"+str(msg.mission_length)+"] len: " +str(len(self.path)))
                    next_wp_geo = self.path[msg.mission_index+1]

                    next_wp_utm = self.coord_conv.geodetic_to_utm(next_wp_geo.latitude, next_wp_geo.longitude)
                    utm_wp = self.coord_conv.geodetic_to_utm(wp_geo.latitude, wp_geo.longitude)
                    head_vec = [utm_wp[3]-next_wp_utm[3], utm_wp[4]-next_wp_utm[4]]
                    vec_degree = 90
                    if not head_vec[0] == 0:
                        vec_degree = math.atan(head_vec[1]/head_vec[0])*180/math.pi #0 equals to east
                    vec_degree += 90
                    vec_degree = 360 - vec_degree
                    if self.path_debug:
                        print "Msg.mission_wp: ", msg.mission_index

                        #print "Vector for heading: ", head_vec
                        #print "Heading on this vector: ", vec_degree


                    self.post_payload['wp_next_hdg_deg'] = vec_degree
                    self.post_payload['wp_next_vel_mps'] = 5
                    pos_utm = self.coord_conv.geodetic_to_utm(GPS_pos.latitude, GPS_pos.longitude)
                    length = math.sqrt((pos_utm[3]-utm_wp[3])**2+(pos_utm[4]-utm_wp[4])**2)
                    time_to_wp = length/msg.ground_speed
                    self.post_payload['wp_next_eta_epoch'] = time.time() + time_to_wp

            self.push_drone_data(self.post_payload)
            self.last_info_pub = time.time()

        self.push_drone_data(self.post_payload)
        #print self.post_payload

    def push_drone_data(self, payload):
        if self.utm_trafic_debug:
            print colored('Trying to POST the data...', 'yellow')
            print payload
        r = ''
        try:
            r = requests.post(url='https://droneid.dk/rmuasd/utm/tracking_data.php', data=payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            if self.utm_trafic_debug:
                print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            if self.utm_trafic_debug:
                print colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:
            if self.utm_trafic_debug:
                print colored('HTTP error', 'red')
                print colored(err, 'yellow')
            # sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print colored('Request error', 'red')
            print colored(err, 'yellow')

            sys.exit(1)
        else:
            if r.text == '1':  # This check can in theory be omitted since the header check should catch an error
                if self.utm_trafic_debug:
                    print colored('Success!\n', 'green')
                    print colored('Status code: %i' % r.status_code, 'yellow')
                    print colored('Content type: %s' % r.headers['content-type'], 'yellow')
                return r.text

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
            print "Entering get_dynamic_NFZ \n"

        self.payload = {
            'data_type': 'dynamic_no_fly'
        }
        r = ''
        try:
            r = requests.get(url='https://droneid.dk/rmuasd/utm/data.php', params=self.payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            if self.utm_trafic_debug:
                print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            if self.utm_trafic_debug:
                print colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:

            print colored('HTTP error', 'red')
            print colored(err, 'yellow')
            # sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            print colored('Request error', 'red')
            print colored(err, 'yellow')
            sys.exit(1)
        else:
            if self.utm_trafic_debug:
                print colored('Status code: %i' % r.status_code, 'yellow')
                print colored('Content type: %s' % r.headers['content-type'], 'yellow')

            data_dict = ''
            try:
                if self.debug:
                    print "Entering the try section of get dynamic NFZ"
                data_dict = json.loads(r.text)


            except:
                print colored('Try part og get dynamic NFZ failed', 'red')
            else:
                if self.debug:
                    print
                    "Succesfully downloaded dynamic NFZ"
                #for entry in data_dict:
                right_now = time.time()
                try:

                    dummy = 1
                except Exception as e:
                    print e
                    rospy.logerr("Failed to retrieve DNFZ, maybe there is none")
                    rospy.logerr(e)
                else:
                    #if self.utm_trafic_debug:
                    #print "DNFZ data from the server: " , data_dict
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
            if self.utm_trafic_debug:
                print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            if self.utm_trafic_debug:
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
            if self.utm_trafic_debug:
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

    def get_drone_data(self):
        debug = False
        if self.debug:
            print "Entering get drone data \n"

        self.payload = {

        }
        r = ''
        try:
            r = requests.get(url='https://droneid.dk/rmuasd/utm/tracking_data.php', params=self.payload, timeout=2)
            r.raise_for_status()
        except requests.exceptions.Timeout:
            # Maybe set up for a retry, or continue in a retry loop
            if self.utm_trafic_debug:
                print colored('Request has timed out', 'red')
        except requests.exceptions.TooManyRedirects:
            # Tell the user their URL was bad and try a different one
            if self.utm_trafic_debug:
                print colored('Request has too many redirects', 'red')
        except requests.exceptions.HTTPError as err:
            if self.utm_trafic_debug:
                print colored('HTTP error', 'red')
                print colored(err, 'yellow')
            # sys.exit(1) # Consider the exit since it might be unintentional in some cases
        except requests.exceptions.RequestException as err:
            # Catastrophic error; bail.
            if self.utm_trafic_debug:
                print colored('Request error', 'red')
                print colored(err, 'yellow')
            sys.exit(1)
        else:
            if self.utm_trafic_debug:
                print colored('Status code: %i' % r.status_code, 'yellow')
                print colored('Content type: %s' % r.headers['content-type'], 'yellow')

            data_dict = ''
            try:
                if self.debug:
                    print "Entering try section of get drone data"
                data_dict = json.loads(r.text)


            except:
                #print colored('Failed to get drone data', 'red')
                self.heart_msg.severity = heartbeat.error
                self.heart_msg.text = 'Failed to get drone data'
            else:
                if self.heart_msg.text == 'Failed to get drone data':
                    self.heart_msg.severity = heartbeat.nothing
                    self.heart_msg.text = ''

                if self.debug:
                    print "Succesfully got drone data"
                try:
                    #print "Drone data: ", data_dict
                    msg = UTMDroneList()
                    i = 1
                    for data in data_dict:
                        if debug:
                            print 'data[' + str(i)+ '/' + str(len(data_dict))+']'
                            i= i+1
                        if self.recent_drone[data['uav_id']] < data['time_epoch']:
                            self.recent_drone[data['uav_id']] = data['time_epoch']                           
                            drone = UTMDrone()
                            drone.next_WP.latitude = data['wp_next_lat_dd']
                            drone.next_WP.longitude = data['wp_next_lng_dd']
                            drone.next_WP.altitude = data['wp_next_alt_m']
                            drone.cur_pos.latitude = data['pos_cur_lat_dd']
                            drone.cur_pos.longitude = data['pos_cur_lng_dd']
                            drone.cur_pos.altitude = data['pos_cur_alt_m']
                            drone.next_vel = data['wp_next_vel_mps']
                            drone.cur_vel = data['pos_cur_vel_mps']
                            drone.next_heading = data['wp_next_hdg_deg']
                            drone.cur_heading = data['pos_cur_hdg_deg']
                            drone.time = data['time_epoch']
                            drone.gps_time = data['pos_cur_gps_timestamp']
                            drone.battery_soc = data['uav_bat_soc']
                            drone.drone_priority = data['uav_op_status']
                            drone.ETA_next_WP = data['wp_next_eta_epoch']
                            drone.drone_id = data['uav_id']
                            msg.drone_list.append(drone)
                    self.utm_drones_pub.publish(msg)
                except Exception as e:
                    print e
                    rospy.logerr("Failed to retrieve drone data, maybe there is none")
                    rospy.logerr(e)
                else:
                    #if self.utm_trafic_debug:
                    #print "DNFZ data from the server: " , data_dict
                    return data_dict

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
                print "Extracting and converting coordinates"

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
        #if self.debug:
        #   print "lower left:", ll_utm
        #   print "upper right: ", utm_ur
        #   print "Delta_y ", delta_y
        if delta_x > delta_y:
            resolution = delta_x / 3000
        else:
            resolution = delta_y / 3000
        self.map_res = resolution

        width = int(delta_x/resolution)
        height = int(delta_y/resolution)
        self.map_width = width
        self.map_height = height
        if self.debug:
            print "About to create empty map with width, height: ", width, height
        self.empty_map = np.zeros((width, height, 1), np.uint8)
        if self.debug:
            print "Created empty map with height, width: ", height, width



    def snfz_into_empty_map(self, utm_coords, upper_right, down_left):
        if self.debug:
            print "Entered SNFZ into empty map"
        snfz_map = self.empty_map
        zone_counter = 0
        #Adding fake Static no fly zone
        fake_geo = [[55.472360, 10.415482, 0], [55.471622, 10.415378, 0], [55.472364, 10.416210, 0], [55.471767, 10.416256, 0]]
        fake_utm_list = []
        for w in fake_geo:
            fake_utm = self.coord_conv.geodetic_to_utm(w[0], w[1])
            fake_utm_list.append(fake_utm)
        utm_coords.append(fake_utm_list)


        fake_geo = [[55.472360, 10.415482, 0], [55.471622, 10.415378, 0], [55.472364, 10.416210, 0],
                    [55.471767, 10.416256, 0]]
        fake_utm_list = []
        for w in fake_geo:
            fake_utm = self.coord_conv.geodetic_to_utm(w[0], w[1])
            fake_utm_list.append(fake_utm)
        utm_coords.append(fake_utm_list)

        for i in utm_coords:
            current_zone = []
            for j in i:

                if j[3] < upper_right[3] and j[3] > down_left[3] and j[4] < upper_right[4] and j[4] > down_left[4]:
                    if self.debug:
                        print "Found SNFZ within map!!"
                    index_width = int((j[3]- down_left[3])/self.map_res)
                    index_heigth = int((j[4]- down_left[4])/self.map_res)
                    #cv2.circle(snfz_map, (index_width, index_heigth), 30, (0, 0, 255), -1)
                    #current_zone_width.append(index_width)
                    current_zone.append([index_width, index_heigth])
                    if self.debug:
                        print "Zone counter: ", zone_counter
                    #if snfz_map[index_width][index_heigth][0] == 255:
                    #    snfz_map[index_width][index_heigth][0] = 255#snfz_map[index_width, index_heigth] = 1
                    if self.debug:
                        print "Found coordinate within map which is NFZ: ", j, "\n Which has index: ", index_width, index_heigth, "which is within zone. ", zone_counter
            if len(current_zone) != 0:
                if self.debug:
                    print "About to make polygon where len is: ", len(current_zone)
                numpy_zone = np.array((current_zone),dtype=np.int32)
                if self.debug:
                    print numpy_zone
                #numpy_zone = numpy_zone.reshape((-1, 1, 2))
                dummy = np.array(([20, 20], [20, 100], [100, 100]), dtype='int32')
                cv2.fillPoly(snfz_map, [numpy_zone], 1)
            zone_counter += 1

            #3 eastern 4 = northing
        #self.show_map(snfz_map)
        if self.debug:
            print "Exited SNFZ into empty map, with map_res: ", self.map_res
            print "Map_res: ", self.map_res
        #self.show_map(snfz_map)
        dilate_width = int(20/self.map_res)
        if self.debug:
            print "Dilate_width: ", dilate_width
        kernel = np.ones((dilate_width, dilate_width), np.uint8)
        snfz_map = cv2.dilate(snfz_map, kernel, iterations=1)
        #self.show_map(snfz_map)
        return snfz_map
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
    #https://stackoverflow.com/questions/34600003/converting-json-to-string-in-python
    def check_dynamic_data(self):
        current_dnfz = self.get_dynamic_nfz()
        if not self.published_first_dnfz:
            message = json.dumps(current_dnfz)
            self.dnfz_pub.publish(message)
            self.published_first_dnfz = 1
        if current_dnfz != self.latest_dynamic_data:
            if self.debug:
                print colored('Difference in latest and current dnfz found' , 'blue')
                print colored("Current dnfz: ", 'blue'), current_dnfz
                print "Latest dnfz: ", self.latest_dynamic_data
            message = json.dumps(current_dnfz)

            self.dnfz_pub.publish(message)
            self.latest_dynamic_data = current_dnfz



def main():
    rospy.init_node('utm_parser')#, anonymous=True)
    rospy.sleep(1)


    par = utm_parser()
    rospy.on_shutdown(par.shutdownHandler)
    #rospy.timer(rospy.Duration(5), par.dnfz_time_callback)
    #
    #par.print_nested_list(par.geoditic_coords)
    #par.print_test_coords()

    #par.get_static_nfz()

    #par.print_zones()
    par.heart_msg.header.frame_id = 'utm_parser'
    par.heart_msg.rate = 1

    while not rospy.is_shutdown():
        rospy.Rate(par.heart_msg.rate).sleep()
        par.heart_msg.header.stamp = rospy.Time.now()
        par.heartbeat_pub.publish(par.heart_msg)
        
        par.check_dynamic_data()
        par.get_drone_data()

if __name__ == "__main__":
    main()

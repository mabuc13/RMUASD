#!/usr/bin/env python3
import rospy
import time
import struct
import rospkg
import datetime
import numpy as np
import rmsd
import copy
import kalman
import kalman_acc3
from utm import utmconv
from math import isnan

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from precision_landing.msg import precland_sensor_data
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
plt.style.use('seaborn-whitegrid')

# defines
RECORDING_FLIGHT_MODE = "Altitude Control"
TAG_ADDRESS = 13

LANDING_TARGET_REF = 0

# 3 floats amount to a length of 12 bytes
LANDING_TARGET_SIZE = 12

# parameters
update_interval = 30

class Analyser(object): 

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        dt = 1/update_interval
        self.state = np.zeros((4,1), float)
        self.kalman = kalman.KalmanFilter(self.state, dt)
        # self.state = np.zeros((6,1), float)
        # self.kalman = kalman3.KalmanFilter(self.state, dt)
        # self.state = np.zeros((9,1), float)
        # self.kalman = kalman_acc3.KalmanFilter(self.state, dt)

        self.local_position_ned = np.array([0,0,0])
        self.sensor_data = np.array([0,0,0]) 
        self.landing_coords = np.array([1.17, 0.75, 0])
        self.new_pos_reading = False
        self.new_sensor_reading = False
        self.vx = 0
        self.vy = 0
        self.vz = 0
 
        self.filtered_pos = np.empty((0,2), float)
        self.landing_target = np.array([0,0,0])

        self.data = np.empty((0,4), float)   
        self.local_data = np.empty((0,3), float)
        # self.filtered_data = np.empty((0,3), float)
        self.filtered_data = np.empty((0,2), float)
        self.rotation_matrix = np.array([[1,0],[0,1]])

        self.fig = plt.gcf()
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.show()
        self.fig.canvas.draw()
        # plt.show()


        # rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_drone_pos)
        # rospy.Subscriber("/telemetry/mission_info", telemetry_mission_info, self.on_mission_info)
        # rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        # rospy.Subscriber("/telemetry/imu_data_ned", telemetry_imu_ned, self.on_imu_data)
        
        rospy.Subscriber("/telemetry/local_position_ned", telemetry_local_position_ned, self.on_local_pos)
        rospy.Subscriber("/landing/sensor_data", precland_sensor_data, self.on_sensor_data)


    def on_sensor_data(self, msg):
        self.new_sensor_reading = True
        self.sensor_data = np.array([msg.x, msg.y, msg.z])

    def on_local_pos(self, msg):
        self.new_pos_reading = True
        self.local_position_ned = np.array([msg.x, msg.y, msg.z])
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    def run(self):
            
        if self.new_pos_reading:
            self.new_pos_reading = False
            # compute something

            plt.scatter(self.local_position_ned[1], self.local_position_ned[0], color='black', s=2) # plot something
            # self.ax.scatter(self.local_position_ned[1], self.local_position_ned[0], -self.local_position_ned[2], color='black', s=2) # plot something
            
            # print(self.local_position_ned)
            # print(self.local_position_ned[0], self.local_position_ned[1])

            # update canvas immediately
            plt.xlim([-10, 10])
            plt.ylim([-10, 10])
            # self.ax.set_zlim(0, 10)
            # plt.pause(0.01)  # I ain't needed!!!
            self.fig.canvas.draw()
            
        if self.new_sensor_reading:
            self.new_sensor_reading = False
            # compute something

            plt.scatter(self.sensor_data[1], self.sensor_data[0], color='red', s=2) # plot something
            # self.ax.scatter(self.sensor_data[1], self.sensor_data[0], self.sensor_data[2], color='red', s=2) # plot something

            relative_target = self.landing_coords - self.sensor_data
            added = relative_target + self.local_position_ned
            # self.ax.scatter(relative_target[1], relative_target[0], -relative_target[2], color='blue', s=2) # plot something


            
            # print(self.sensor_data)
            # print(self.sensor_data[0], self.sensor_data[1])

            # update canvas immediately
            plt.xlim([-10, 10])
            plt.ylim([-10, 10])
            # self.ax.set_zlim(0, 10)
            #plt.pause(0.01)  # I ain't needed!!!
            self.fig.canvas.draw()


    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

if __name__ == "__main__":
    rospy.init_node('bag_analyser')
    rospy.sleep(1)

    al = Analyser()

    rospy.on_shutdown(al.shutdownHandler)
        
    while not rospy.is_shutdown():
        al.run()

        al.rate.sleep()

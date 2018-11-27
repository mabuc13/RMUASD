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
from smbus2 import SMBus
from math import isnan

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from node_monitor.msg import heartbeat

# defines
RECORDING_FLIGHT_MODE = "Altitude Control"
TAG_ADDRESS = 13

LANDING_TARGET_REF = 0

# 3 floats amount to a length of 12 bytes
LANDING_TARGET_SIZE = 12

# parameters
update_interval = 30

def rotation_matrix(sigma):
    """

    https://en.wikipedia.org/wiki/Rotation_matrix

    """

    radians = sigma * np.pi / 180.0

    r11 = np.cos(radians)
    r12 = -np.sin(radians)
    r21 = np.sin(radians)
    r22 = np.cos(radians)

    R = np.array([[r11, r12], [r21, r22]])

    return R

class PrecisionLanding(object): 

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        self.bus = SMBus(1)
        self.utmconv = utmconv()
        self.recording = False

        self.main_mode = ""
        self.sub_mode = ""
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.mission_idx = 0
        self.mission_len = 0

        dt = 1/update_interval
        self.state = np.zeros((4,1), float)
        self.kalman = kalman.KalmanFilter(self.state, dt)
        # self.state = np.zeros((6,1), float)
        # self.kalman = kalman3.KalmanFilter(self.state, dt)
        # self.state = np.zeros((9,1), float)
        # self.kalman = kalman_acc3.KalmanFilter(self.state, dt)

        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
 
        self.filtered_pos = Point()
        self.local_drone_pos = Point()
        self.landing_target = Point()
        self.landing_coords = Point(1.17, 0.75, 0)

        self.data = np.empty((0,4), float)   
        self.local_data = np.empty((0,3), float)
        # self.filtered_data = np.empty((0,3), float)
        self.filtered_data = np.empty((0,2), float)
        self.rotation_matrix = np.array([[1,0],[0,1]])

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("precision_landing")
        self.data_path = package_path + "/data/"

        rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_drone_pos)
        rospy.Subscriber("/telemetry/mission_info", telemetry_mission_info, self.on_mission_info)
        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/telemetry/imu_data_ned", telemetry_imu_ned, self.on_imu_data)
        rospy.Subscriber("/telemetry/local_position_ned", telemetry_local_position_ned, self.on_local_pos)
        

        self.landing_target_pub = rospy.Publisher("/telemetry/set_landing_target", telemetry_landing_target, queue_size=0)

    def on_drone_pos(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

        # if self.recording:
        #     # rospy.loginfo("Pos!")
        #     if self.get_landing_target():
        #         (_, _, _, easting, northing) = self.utmconv.geodetic_to_utm(self.lat, self.lon)
        #         self.data = np.append(self.data, np.array([[easting, northing, self.local_drone_pos.x, self.local_drone_pos.y]]), axis=0)
        #         self.local_data = np.append(self.local_data, np.array([[self.local_drone_pos.x, self.local_drone_pos.y, self.local_drone_pos.z]]), axis=0)

        #         msg = telemetry_landing_target(
        #             landing_target=self.landing_target
        #         )
        #         self.landing_target_pub.publish(msg)

    def on_mission_info(self, msg):
        self.mission_idx = msg.active_waypoint_idx
        self.mission_len = msg.active_mission_len

    def on_local_pos(self, msg):
        self.new_vel_reading = True
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    def on_imu_data(self, msg):
        self.new_imu_reading = True
        self.ax = msg.ax
        self.ay = msg.ay
        self.az = msg.az

        # if self.recording:
        #     if self.get_landing_target():
        #         # measurement = np.array([[self.local_drone_pos.x], [self.local_drone_pos.y], [self.local_drone_pos.z],
        #         #     [0], [0], [0], [msg.ax], [msg.ay], [msg.az + 9.82]])
        #         measurement = np.array([[self.local_drone_pos.x], [self.local_drone_pos.y]])
        #         self.state = self.kalman.update(measurement)
        #         self.local_data = np.append(self.local_data, np.array([[self.local_drone_pos.x, self.local_drone_pos.y, self.local_drone_pos.z]]), axis=0)

        #         msg = telemetry_landing_target(
        #             landing_target=self.landing_target
        #         )
        #         self.landing_target_pub.publish(msg)
        #     else:
        #         self.state = self.kalman.update()

        #     self.filtered_pos = Point(self.state[0,0], self.state[1,0], self.state[2,0])
        #     # self.filtered_data = np.append(self.filtered_data, np.array([[self.filtered_pos.x, self.filtered_pos.y, self.filtered_pos.z]]), axis=0)
        #     self.filtered_data = np.append(self.filtered_data, np.array([[self.filtered_pos.x, self.filtered_pos.y]]), axis=0)

    def on_heartbeat_status(self, msg):
        # save the data when switching out of position mode
        if self.main_mode == RECORDING_FLIGHT_MODE and msg.main_mode != RECORDING_FLIGHT_MODE:
            now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            savename1 = self.data_path + now + "_filtered_data.csv"
            savename2 = self.data_path + now + "_local_data.csv"

            # copy data array to prevent other callbacks from screwing things up
            # temp = copy.copy(self.data)

            # temp[:,0:2] -= rmsd.centroid(temp[:,0:2])
            # temp[:,2:4] -= rmsd.centroid(temp[:,2:4])

            # B = np.dot(temp[:,0:2], rotation_matrix(90))

            # self.rotation_matrix = rmsd.kabsch(temp[:,0:2], B)
            # rospy.logwarn(self.rotation_matrix)
            np.savetxt(savename1, self.filtered_data, delimiter=',')
            self.filtered_data = np.empty((0,2), float)
            np.savetxt(savename2, self.local_data, delimiter=',')
            self.local_data = np.empty((0,3), float)

        self.main_mode = msg.main_mode
        self.sub_mode = msg.sub_mode

    def get_landing_target(self):
        try:
            data = self.bus.read_i2c_block_data(TAG_ADDRESS, LANDING_TARGET_REF, LANDING_TARGET_SIZE)
            (x,y,z) = struct.unpack('<fff',bytearray(data))

            # Make sure that no nans are accepted as values
            if isnan(x) or isnan(y):
                return False
            
            if isnan(z):
                z = 0.0

            self.local_drone_pos = Point(x, y, z)
            self.landing_target = Point(self.landing_coords.x-x, self.landing_coords.y -y, self.landing_coords.z-z)
            # print(self.landing_target)
            return True
        except Exception as e:
            rospy.logwarn(e)
            return False

    def run(self):
        if self.main_mode == RECORDING_FLIGHT_MODE:
            self.recording = True
        else:
            self.recording = False

        if self.recording:
            # if self.new_imu_reading:
            #     self.new_imu_reading = False
            
            if self.new_vel_reading:
                self.new_vel_reading = False
                if self.get_landing_target():
                    # update kalman filter with both position and velocity
                    measurement = np.array([[self.local_drone_pos.x], [self.local_drone_pos.y], [self.vx], [self.vy]])
                    self.state = self.kalman.update(measurement, kalman.Measurement.BOTH)
                else:
                    # update kalman filter only with velocity
                    measurement = np.array([[self.vx], [self.vy]])
                    self.state = self.kalman.update(measurement, kalman.Measurement.VEL)

            else:
                if self.get_landing_target():
                    # update kalman filter with position
                    measurement = np.array([[self.local_drone_pos.x], [self.local_drone_pos.y]])
                    self.state = self.kalman.update(measurement, kalman.Measurement.POS)
                else:
                    # only do kalman prediction
                    self.state = self.kalman.update()

            self.filtered_pos = Point(self.state[0,0], self.state[1,0])  
            self.filtered_data = np.append(self.filtered_data, np.array([[self.filtered_pos.x, self.filtered_pos.y]]), axis=0)


        # activate landing target
        if self.sub_mode == "Mission":
            if self.mission_len > 0:
                if self.mission_idx == self.mission_len - 1:
                    if self.get_landing_target():
                        msg = telemetry_landing_target(
                            landing_target=self.landing_target
                        )
                        self.landing_target_pub.publish(msg)


    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

if __name__ == "__main__":
    rospy.init_node('precision_landing')
    rospy.sleep(1)

    pl = PrecisionLanding()

    rospy.on_shutdown(pl.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'precision_landing'
    heart_msg.rate = update_interval
        
    while not rospy.is_shutdown():
        pl.run()

        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        pl.rate.sleep()

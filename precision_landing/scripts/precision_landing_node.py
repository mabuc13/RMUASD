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
from precision_landing.msg import precland_sensor_data

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
        self.new_imu_reading = False
        self.new_vel_reading = False

        dt = 1/update_interval
        self.state = np.zeros((4,1), float)
        self.kalman = kalman.KalmanFilter(self.state, dt)
        # self.state = np.zeros((6,1), float)
        # self.kalman = kalman3.KalmanFilter(self.state, dt)
        # self.state = np.zeros((9,1), float)
        # self.kalman = kalman_acc3.KalmanFilter(self.state, dt)

        self.local_position_ned = np.array([0,0,0])
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.ax = 0
        self.ay = 0
        self.az = 0
 
        self.filtered_pos = np.empty((0,2), float)
        self.local_position_landing = Point()
        self.landing_target = np.array([0,0,0])
        self.landing_coords = Point(1.5, 1.75, 0)

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
        
        self.sensor_data_pub    = rospy.Publisher("/landing/sensor_data", precland_sensor_data, queue_size=0)
        self.landing_target_pub = rospy.Publisher("/telemetry/set_landing_target", telemetry_landing_target, queue_size=0)

    def on_drone_pos(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt


    def on_mission_info(self, msg):
        self.mission_idx = msg.active_waypoint_idx
        self.mission_len = msg.active_mission_len

    def on_local_pos(self, msg):
        self.new_vel_reading = True
        self.local_position_ned = np.array([msg.x, msg.y, msg.z])
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz

    def on_imu_data(self, msg):
        self.new_imu_reading = True
        self.ax = msg.ax
        self.ay = msg.ay
        self.az = msg.az

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
            # Compensating for other old xyz coordinates
            (y,x,z) = struct.unpack('<fff',bytearray(data))

            # print(x,y,z)
            # Make sure that no nans are accepted as values
            if isnan(x) or isnan(y) or isnan(z):
                return False
            
            # if isnan(z):
            #     z = 0.0

            sensor_data = precland_sensor_data(x, y, z)
            
            self.sensor_data_pub.publish(sensor_data)
            self.local_position_landing = Point(x, y, -z)
            self.relative_target = np.array([self.landing_coords.x-x, self.landing_coords.y -y, self.landing_coords.z-(-z)])   
            self.landing_target = self.local_position_ned + self.relative_target
            return True
        except Exception as e:
            rospy.logwarn(e)
            return False

    def run(self):
        if self.main_mode == RECORDING_FLIGHT_MODE:
            self.recording = True
        else:
            self.recording = False

        # print(self.main_mode)

        # if self.main_mode == "Stabilized" and self.local_data.shape[0] > 0:
        #     now = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        #     savename2 = self.data_path + now + "_local_data.csv"
        #     np.savetxt(savename2, self.local_data, delimiter=',')
        #     self.local_data = np.empty((0,3), float)
        #     rospy.logwarn("SAVING!")


        # if self.recording:
            # if self.new_imu_reading:
            #     self.new_imu_reading = False
            
        if self.get_landing_target():
            # self.data = np.append(self.data, np.array([[easting, northing, self.local_position_landing.x, self.local_position_landing.y]]), axis=0)
            # self.local_data = np.append(self.local_data, np.array([[self.local_position_landing.x, self.local_position_landing.y, self.local_position_landing.z]]), axis=0)

            msg = telemetry_landing_target(
                landing_target=Point(
                    self.landing_target[0], 
                    self.landing_target[1], 
                    self.landing_target[2]
                )
            )
            self.landing_target_pub.publish(msg)
            print(self.local_position_landing)

        # if self.new_vel_reading:
        #     self.new_vel_reading = False
        #     if self.get_landing_target():
        #         # update kalman filter with both position and velocity
        #         measurement = np.array([[self.local_position_landing.x], [self.local_position_landing.y], [self.vx], [self.vy]])
        #         self.state = self.kalman.update(measurement, kalman.Measurement.BOTH)
        #     else:
        #         # update kalman filter only with velocity
        #         measurement = np.array([[self.vx], [self.vy]])
        #         self.state = self.kalman.update(measurement, kalman.Measurement.VEL)

        # else:
        #     if self.get_landing_target():

        #         # update kalman filter with position
        #         measurement = np.array([[self.local_position_landing.x], [self.local_position_landing.y]])
        #         self.state = self.kalman.update(measurement, kalman.Measurement.POS)
        #     else:
        #         # only do kalman prediction
        #         self.state = self.kalman.update()
        #         pass

        # print(self.state[0,0], self.state[1,0])
        # print(self.local_position_landing.x, self.local_position_landing.y)

        if self.recording:
            if self.get_landing_target():
                self.local_data = np.append(self.local_data, np.array([[self.local_position_landing.x, self.local_position_landing.y, self.local_position_landing.z]]), axis=0)
                self.filtered_pos = np.array([[ self.state[0,0], self.state[1,0] ]])  
                self.filtered_data = np.append(self.filtered_data, self.filtered_pos, axis=0)

 
        # activate landing target
        if self.sub_mode == "Mission":
            if self.mission_len > 0:
                if self.mission_idx == self.mission_len - 1:
                    if self.get_landing_target():
                        msg = telemetry_landing_target(
                            landing_target=Point(
                                self.landing_target[0], 
                                self.landing_target[1], 
                                self.landing_target[2]
                            )
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

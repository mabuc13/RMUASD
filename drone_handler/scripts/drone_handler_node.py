#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import struct
from datetime import datetime
import drone

from gcs.msg import * # pylint: disable=W0614
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.msg import telemetry_heartbeat_status, telemetry_mav_mode, telemetry_mission_info, telemetry_statustext
from mavlink_lora.msg import mavlink_lora_attitude, mavlink_lora_pos, mavlink_lora_status

# parameters
update_interval = 10

class DroneHandler(object):

    def __init__(self):
        
        # hardcode the drone ID to 1 which is the drone we will be using
        self.drones = {1: drone.Drone()}

		# status variables
        rospy.sleep (1) # wait until everything is running

        self.boot_time = rospy.Time.now().to_sec() * 1000

        # Service handlers
        self.mission_request_service = rospy.Service("/drone_handler/mission_request", Trigger, self.mission_request, buff_size=10)

        # Topic handlers
        # self.mavlink_msg_pub        = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        rospy.Subscriber("/gcs/PathRequest", DronePath, self.on_drone_path)
        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/telemetry/mav_mode", telemetry_mav_mode, self.on_mav_mode)
        rospy.Subscriber("/telemetry/statustext", telemetry_statustext, self.on_statustext)
        rospy.Subscriber("/telemetry/mission_info", telemetry_mission_info, self.on_mission_info)
        rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_drone_pos)
        rospy.Subscriber("/mavlink_attitude", mavlink_lora_attitude, self.on_drone_attitude)
        rospy.Subscriber("/mavlink_status", mavlink_lora_status, self.on_drone_status)

        # rospy.Timer(rospy.Duration(0.1), self.mission_info_cb)

        self.rate = rospy.Rate(update_interval)

    def on_drone_path(self, msg):
        pass

    def on_heartbeat_status(self, msg):
        # print(msg)
        pass

    def on_mav_mode(self, msg):
        pass

    def on_statustext(self, msg):
        pass

    def on_mission_info(self, msg):
        pass

    def on_drone_attitude(self, msg):
        pass

    def on_drone_status(self, msg):
        pass

    def on_drone_pos(self, msg):
        pass


    def mission_request(self, srv):
        pass

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")



if __name__ == "__main__":    
    rospy.init_node('drone_handler')#, anonymous=True)
    rospy.sleep(1)

    dh = DroneHandler()

    rospy.on_shutdown(dh.shutdownHandler)

    rospy.spin()

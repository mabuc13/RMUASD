#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import struct

from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos, mavlink_lora_status, mavlink_lora_mission_list, mavlink_lora_command_ack
from gcs.msg import * # pylint: disable=W0614
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.srv import SetMode, ChangeSpeed, UploadFromFile, TakeoffDrone, LandDrone
from telemetry.msg import telemetry_statustext, telemetry_heartbeat_status, telemetry_mav_mode, telemetry_mission_info
from mavlink_defines import * # pylint: disable=W0614
from datetime import datetime
import mission_handler
import command_handler

# parameters
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
update_interval = 10

class Telemetry(object):

    def __init__(self):
        self.request_sent = False
        self.first_msg_ok = False

		# status variables
        self.batt_volt = 0.0
        self.last_heard = 0
        self.last_heard_sys_status = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.local_lat = 0.0
        self.local_lon = 0.0
        self.local_alt = -5.0
        self.active_waypoint = 0
        self.mission_length = 0
        self.recorded_sys_id = 0
        self.recorded_comp_id = 0

        rospy.sleep (1) # wait until everything is running

        self.boot_time = rospy.Time.now().to_sec() * 1000

        # Service handlers
        self.arm_service                = rospy.Service("/telemetry/arm_drone", Trigger, self.command_handler.arm_drone, buff_size=10)

        self.upload_proxy   = rospy.ServiceProxy("/telemetry/upload_mission_from_file", UploadFromFile)

        # Topic handlers
        rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)
        rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, self.on_mavlink_lora_status)
        rospy.Subscriber("/telemetry/mission_info", telemetry_mission_info, self.on_mission_info)
        
        # rospy.Timer(rospy.Duration(0.1), self.mission_info_cb)
        
        self.rate = rospy.Rate(update_interval)


    def on_mission_info(self, msg):
        self.active_waypoint = msg.active_waypoint_idx
        self.mission_length = msg.mission_length

    def on_mavlink_lora_pos(self,msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

    def on_mavlink_lora_status (self, msg):
        self.last_heard = msg.last_heard.secs + msg.last_heard.nsecs/1.0e9
        self.last_heard_sys_status = msg.last_heard_sys_status.secs + msg.last_heard_sys_status.nsecs/1.0e9
        self.batt_volt = msg.batt_volt / 1000.0

    def on_keypress(self, msg):
        pass

    def mission_info_cb(self, event):
        msg = telemetry_mission_info(
            system_id=self.recorded_sys_id,
            component_id=self.recorded_comp_id,
            timestamp=rospy.Time.now(),
            active_waypoint_idx=self.mission_handler.active_mission_item,
            mission_length=self.mission_handler.active_mission_length            
        )
        self.mission_info_pub.publish(msg)

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

def main():
    rospy.init_node('telemetry')#, anonymous=True)
    rospy.sleep(1)

    tel = Telemetry()

    rospy.on_shutdown(tel.shutdownHandler)
    # tel.enable_rc_channels()
    # Send global setpoint every 0.4 seconds
    # rospy.Timer(rospy.Duration(0.2),tel.send_local_setpoint)

    rospy.spin()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import struct

from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos, mavlink_lora_status
# from gcs.msg import *
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.srv import SetMode
from telemetry.msg import telemetry_statustext, telemetry_heartbeat_status, telemetry_mav_mode
from mavlink_defines import *
from datetime import datetime

# parameters
mavlink_lora_pos_sub_topic = '/mavlink_pos'
update_interval = 10

class GPS_Collector(object):

    def __init__(self):
        self.request_sent = False
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        rospy.sleep (1) # wait until everything is running

        rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)
        self.rate = rospy.Rate(update_interval)

    def on_mavlink_lora_pos(self,msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

        with open("gps.csv", "a") as file:
            file.write("{},{}\n".format(self.lat,self.lon))
            print("{},{}\n".format(self.lat,self.lon))


    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")



def main():
    rospy.init_node('gps_collector')#, anonymous=True)
    rospy.sleep(1)

    gps = GPS_Collector()

    rospy.spin()

if __name__ == "__main__":
    main()
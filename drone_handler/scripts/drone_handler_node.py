#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import struct

from gcs.msg import * # pylint: disable=W0614
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger, TriggerResponse
from datetime import datetime

# parameters
update_interval = 10



class DroneHandler(object):

    def __init__(self):
        self.request_sent = False
        self.first_msg_ok = False

		# status variables
        rospy.sleep (1) # wait until everything is running

        self.boot_time = rospy.Time.now().to_sec() * 1000

        # Service handlers
        self.mission_request_service = rospy.Service("/drone_handler/mission_request", Trigger, self.on_mission_request, buff_size=10)

        # Topic handlers
        # self.mavlink_msg_pub        = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        rospy.Subscriber("/mavlink_interface/command/ack", mavlink_lora_command_ack, self.command_handler.on_command_ack)

        # rospy.Timer(rospy.Duration(0.1), self.mission_info_cb)

        self.rate = rospy.Rate(update_interval)

    def on_mission_request(self, srv):
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

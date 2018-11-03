#!/usr/bin/env python3

import signal
import sys
import rospy
import time

from gcs.msg import * # pylint: disable=W0614
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import mavlink_lora_attitude, mavlink_lora_pos, mavlink_lora_status, mavlink_lora_mission_ack
from gcs.msg import DroneInfo, GPS, NiceInfo

import rosnode 

 
# defines

# parameters
update_interval = 10

class NodeMonitor(object):

    def __init__(self):

		# status variables
        rospy.sleep (1) # wait until everything is running

        self.boot_time = rospy.Time.now().to_sec() * 1000

        # Service handlers
        # self.mission_request_service = rospy.Service("/drone_handler/mission_request", Trigger, self.mission_request, buff_size=10)

        # Topic handlers
        # self.mavlink_msg_pub        = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        self.drone_info_pub     = rospy.Publisher("/drone_handler/DroneInfo", DroneInfo, queue_size=0) 
        self.nice_info_pub      = rospy.Publisher("/drone_handler/NiceInfo", NiceInfo, queue_size=0) 

        rospy.Subscriber("/gcs/PathRequest", DronePath, self.on_drone_path)
        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/telemetry/mav_mode", telemetry_mav_mode, self.on_mav_mode)
        rospy.Subscriber("/telemetry/statustext", telemetry_statustext, self.on_statustext)
        rospy.Subscriber("/telemetry/mission_info", telemetry_mission_info, self.on_mission_info)
        rospy.Subscriber("/telemetry/vfr_hud", telemetry_vfr_hud, self.on_vfr_hud)
        rospy.Subscriber("/telemetry/home_position", telemetry_home_position, self.on_home_position)
        rospy.Subscriber("/telemetry/cmd_retry_fail", telemetry_cmd_retry_fail, self.on_cmd_fail)
        rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_drone_pos)
        rospy.Subscriber("/mavlink_attitude", mavlink_lora_attitude, self.on_drone_attitude)
        rospy.Subscriber("/mavlink_status", mavlink_lora_status, self.on_drone_status)

        # rospy.Timer(rospy.Duration(0.1), self.mission_info_cb)

        self.rate = rospy.Rate(update_interval)

    def on_cmd_fail(self, msg):
        pass

    def on_drone_path(self, msg):
        pass

    def on_heartbeat_status(self, msg):
        pass

    def on_mav_mode(self, msg):
        pass
        
    def on_statustext(self, msg):
        pass

    def on_vfr_hud(self, msg):
        pass

    def on_home_position(self, msg):
        pass

    def on_mission_info(self, msg):
        pass


    def on_drone_attitude(self, msg):
        pass

    def on_drone_status(self, msg):
        pass

    def on_drone_pos(self, msg):
        pass

    def run(self, event):
        pass

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")



if __name__ == "__main__":

    rospy.init_node("A", anonymous=True)
    rospy.sleep (1)

    rosnode.rosnode_ping("telemetry",max_count=3, verbose=True)
    
    rospy.spin()

    # rospy.init_node('node_monitor')#, anonymous=True)
    # rospy.sleep(1)

    # nm = NodeMonitor()
    
    # rospy.on_shutdown(nm.shutdownHandler)

    # rospy.spin()

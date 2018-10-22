#!/usr/bin/env python3

import signal
import sys
import rospy
import rospkg
import time
import struct

from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos, mavlink_lora_status
from gcs.msg import *
from std_msgs.msg import Int8,Header
from std_srvs.srv import Trigger, TriggerResponse
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list, mavlink_lora_mission_partial_list
from mavlink_defines import *
from datetime import datetime
from import_mission_plan import import_plan
from telemetry.srv import UploadFromFile, UploadFromFileResponse

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
update_interval = 10

class MissionHandler(object):

    def __init__(self):
        self.request_sent = False

		# status variables
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.local_lat = 0.0
        self.local_lon = 0.0
        self.local_alt = -5.0

        self.mission_list = mavlink_lora_mission_list()
        self.valid_mission = False

        self.upload_timer = None

        rospy.sleep (1) # wait until everything is running

        # Service handlers
        self.mission_upload_service = rospy.Service("/mission/upload", Trigger, self.upload_mission, buff_size=10)
        self.mission_upload_partial_service = rospy.Service("/mission/partial", Trigger, self.upload_partial_mission, buff_size=10)
        self.mission_upload_from_file_service = rospy.Service("/mission/upload_from_file", UploadFromFile, self.upload_from_file, buff_size=10)

        # Topic handlers
        self.mission_upload_pub = rospy.Publisher("mavlink_interface/mission/mavlink_upload_mission", mavlink_lora_mission_list, queue_size=0)
        self.partial_mission_upload_pub = rospy.Publisher("mavlink_interface/mission/mavlink_upload_partial_mission", mavlink_lora_mission_partial_list, queue_size=0)
        self.mavlink_msg_pub = rospy.Publisher("/mavlink_tx", mavlink_lora_msg,queue_size=0)

        rospy.Subscriber("/downloaded_mission", mavlink_lora_mission_list, self.on_mission_list)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
        self.rate = rospy.Rate(update_interval)

    def on_mavlink_msg(self,msg):                
        if msg.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST or msg.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST_INT:
            pass

    def on_mission_list(self, msg):
        self.waypoints = msg.waypoints

    def upload_mission(self, srv):
        msg = mavlink_lora_mission_list(
            header=Header(stamp=rospy.Time.now()),
            waypoints=self.waypoints
        )
        self.mission_upload_pub.publish(msg)
        return TriggerResponse()

    def upload_from_file(self, srv):
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("telemetry")
        plan_path = package_path + "/plans/"
        try:
            msg = import_plan(plan_path + srv.filename, target_sys=1, target_comp=0)
            self.mission_upload_pub.publish(msg)
        except Exception as e:
            rospy.logerr("Could not import plan from file")
            rospy.logerr(e)
            
        return UploadFromFileResponse()

    def upload_partial_mission(self, srv):
        msg = mavlink_lora_mission_partial_list(
            start_index=4,
            end_index=4
        )
        
        msg.waypoints.append(self.waypoints[1])
        self.partial_mission_upload_pub.publish(msg)
        
        mav_msg = mavlink_lora_msg(
            msg_id=MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST,
            payload_len=6,
            sys_id=0,
            comp_id=0
        )
        mav_msg.payload = struct.pack('<hhBB', 4, 4, 1, 0)
        self.mavlink_msg_pub.publish(mav_msg)

        return TriggerResponse()

def main():
    rospy.init_node('mission_node')#, anonymous=True)
    rospy.sleep(1)

    mh = MissionHandler()

    rospy.spin()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

# imports
from sys import argv
import rospy
import rospkg
from datetime import datetime
from mavlink_defines import * # pylint: disable=W0614
from mission_lib import * # pylint: disable=W0614
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.srv import UploadFromFile, UploadFromFileResponse, UploadMissionResponse
from telemetry.msg import telemetry_mission_info
from import_mission_plan import import_plan

# defines
MAX_RETRIES = 5
TIMEOUT = 1.5

MAV_MISSION_ACCEPTED = 0
MAV_MISSION_ERROR = 1

class MissionHandler(object):

    def __init__(self):
        # initiate variables
        self.stop = False
        self.first_msg_ok = False
        self.request_sent = False
        self.busy = False
        self.mission_count = 0
        self.mission_id_next = 0
        self.mi = mission_lib()
        self.sys_id = 0  # reset when receiving first msg
        self.comp_id = 0
        self.active_mission_item = 0
        self.active_mission_length = 0

        self.mission_list_down  = mavlink_lora_mission_list()
        self.mission_list_up    = mavlink_lora_mission_list()
        self.active_mission     = mavlink_lora_mission_list(
            waypoints=[mavlink_lora_mission_item_int()]
        )
        # self.active_mission.waypoints.append(mavlink_lora_mission_item_int())

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("telemetry")
        self.plan_path = package_path + "/plans/"

        self.download_timer = None
        self.retries = 0

        # Topic handlers
        self.mavlink_msg_pub        = rospy.Publisher("/mavlink_tx", mavlink_lora_msg, queue_size=0)
        self.mission_upload_pub     = rospy.Publisher("/mavlink_interface/mission/mavlink_upload_mission", mavlink_lora_mission_list, queue_size=0)
        self.mission_info_pub       = rospy.Publisher("/telemetry/mission_info", telemetry_mission_info, queue_size=0)
    def on_mavlink_msg(self, msg):
        if self.first_msg_ok == False:
            self.first_msg_ok = True
            self.sys_id = msg.sys_id
            self.mi.set_target(self.sys_id, self.comp_id)

        if msg.msg_id == MAVLINK_MSG_ID_MISSION_ITEM:
            # stop the timeout
            self.download_timer.shutdown()
            # add the item to the list
            mission_item = self.mi.unpack_mission_item(msg.payload)
            # print(mission_item)
            # check if the waypoint is valid
            if mission_item[8] != 530:
                mission_item_msg = mavlink_lora_mission_item_int(
                    param1=mission_item[0],
                    param2=mission_item[1],
                    param3=mission_item[2],
                    param4=mission_item[3],
                    x=int(mission_item[4]*1e7),
                    y=int(mission_item[5]*1e7),
                    z=mission_item[6],
                    seq=mission_item[7],
                    command=mission_item[8],
                    current=mission_item[9],
                    autocontinue=mission_item[10],
                    target_system=self.sys_id,
                    target_component=self.comp_id,
                    frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                )

                self.mission_list_down.waypoints.append(mission_item_msg)

            if self.mission_id_next < self.mission_count - 1:
                self.mission_id_next += 1
                # if self.mission_id_next == 10:
                # 	self.mission_id_next += 2
                rospy.loginfo("Requesting item #%d" % self.mission_id_next)
                self.send_mavlink_mission_req(self.mission_id_next)
            else:
                # finished download
                rospy.loginfo("Download has finished.")
                self.mission_list_down.header.stamp = rospy.Time.now()
                self.mission_id_next = 0
                self.busy = False
                self.active_mission_length = len(self.mission_list_down.waypoints)
                self.active_mission = self.mission_list_down
                self.mi.pack_mission_ack(MAV_MISSION_ACCEPTED)
                self.mavlink_msg_pub.publish(self.mi.msg)
                # self.mission_list_pub.publish(self.mission_list_down)

        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_COUNT:
            # stop the timeout
            self.download_timer.shutdown()

            self.mission_count = self.mi.unpack_mission_count(msg.payload)
            rospy.loginfo("Mission count: %d" % self.mission_count)
            if self.mission_count > 0:
                self.mission_id_next = 0
                rospy.loginfo("Requesting item #%d" % self.mission_id_next)
                self.send_mavlink_mission_req(self.mission_id_next)
            else:
                self.busy = False

        # elif msg.msg_id == MAVLINK_MSG_ID_MISSION_ACK:
        #     self.busy = False
        #     self.active_mission_length = len(self.mission_list_up.waypoints)
        #     self.active_mission = self.mission_list_up

        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_CURRENT:
            self.active_mission_item = self.mi.unpack_mission_current(msg.payload)

            try:
                current = self.active_mission.waypoints[self.active_mission_item]
            except IndexError:
                current = mavlink_lora_mission_item_int()

            msg = telemetry_mission_info(
                system_id=msg.sys_id,
                component_id=msg.comp_id,
                timestamp=rospy.Time.now(),
                active_waypoint_idx=self.active_mission_item,
                active_mission_len=self.active_mission_length,
                current_item=current
            )
            self.mission_info_pub.publish(msg)


    def on_mission_list(self, msg):
        self.mission_list_up.waypoints = msg.waypoints

    def on_mission_ack(self, msg):
        self.busy = False
        self.retries = 0
        self.active_mission_length = len(self.mission_list_up.waypoints)
        self.active_mission = self.mission_list_up
        rospy.loginfo(msg.result_text)

    def send_mavlink_mission_req_list(self):
        self.mi.msg.header.stamp = rospy.Time.now()
        self.mi.pack_mission_req_list()
        self.mavlink_msg_pub.publish(self.mi.msg)
        self.mission_list_down.waypoints.clear()

        # start timeout
        self.download_timer = rospy.Timer(
            period=rospy.Duration(TIMEOUT),
            callback=self.request_list_timeout,
            oneshot=True
        )

    def send_mavlink_mission_req(self, mission_id):
        self.mi.msg.header.stamp = rospy.Time.now()
        self.mi.pack_mission_req(mission_id)
        self.mavlink_msg_pub.publish(self.mi.msg)

        # start timeout
        self.download_timer = rospy.Timer(
            period=rospy.Duration(TIMEOUT),
            callback=self.request_item_timeout,
            oneshot=True
        )

    def request_list_timeout(self, event):
        self.retries += 1
        rospy.loginfo("List request timeout!")

        if self.retries > MAX_RETRIES:
            self.retries = 0
            rospy.logwarn("Too many failures - cancelling download")
            self.busy = False
        else:
            rospy.loginfo("Requesting list again - %d. retry" %
                            self.retries)
            self.send_mavlink_mission_req_list()

    def request_item_timeout(self, event):
        self.retries += 1
        rospy.loginfo("Item request timeout!")

        if self.retries > MAX_RETRIES:
            self.retries = 0
            rospy.logwarn("Too many failures - cancelling download")
            self.busy = False
        else:
            rospy.loginfo("Requesting item #%d again" %
                            self.mission_id_next)
            self.send_mavlink_mission_req(self.mission_id_next)

    def download(self, srv):
        if self.busy:
            rospy.logwarn("Can't download mission. Handler is busy.")
            return TriggerResponse(False, "Can't download mission. Handler is busy.")
        else:
            self.busy = True
            self.send_mavlink_mission_req_list()
            return TriggerResponse(True, "Downloading mission.")

    def upload(self, srv):
        if self.busy:
            rospy.logwarn("Can't upload mission. Handler is busy.")
            return UploadMissionResponse(False, "Can't upload mission. Handler is busy.")
        else:
            rospy.loginfo("Upload started.")
            self.busy = True
            self.mission_list_up.header.stamp = rospy.Time.now()
            self.mission_list_up.waypoints = srv.waypoints
            self.mission_upload_pub.publish(self.mission_list_up)
            return UploadMissionResponse(True, "Uploading mission.")

    def upload_from_file(self, srv):
        try:
            mission_list = import_plan(self.plan_path + srv.filename, target_sys=1, target_comp=0)
        except Exception as e:
            rospy.logerr("Could not import plan from file")
            rospy.logerr(e)
            return UploadFromFileResponse(False, e)

        if self.busy:
            rospy.logwarn("Can't upload mission. Handler is busy.")
            return UploadFromFileResponse(False, "Can't upload mission. Handler is busy.")
        else:
            rospy.loginfo("Upload started.")
            self.busy = True
            mission_list.header.stamp = rospy.Time.now()
            self.mission_list_up = mission_list
            self.mission_upload_pub.publish(mission_list)
            return UploadFromFileResponse(True, "Uploading mission.")

    def mission_set_current(self, msg):
        message = mavlink_lora_msg()
        message.msg_id = MAVLINK_MSG_ID_MISSION_SET_CURRENT
        message.payload_len = MAVLINK_MSG_ID_MISSION_SET_CURRENT_LEN
        message.payload = struct.pack('<HBB', msg.data, self.mi.target_sys, self.mi.target_comp)
        # message.payload = struct.pack('<HBB', msg.data, 0, 0)

        self.mavlink_msg_pub.publish(message)
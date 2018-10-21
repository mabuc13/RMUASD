
# imports
from sys import argv
import rospy
import rospkg
from mavlink_defines import *
from mission_lib import *
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.srv import UploadFromFile, UploadFromFileResponse
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

        self.mission_list_down  = mavlink_lora_mission_list()
        self.mission_list_up    = mavlink_lora_mission_list()

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("telemetry")
        self.plan_path = package_path + "/plans/"

        self.download_timer = None
        self.retries = 0

        # Topic handlers
        self.mavlink_msg_pub        = rospy.Publisher("/mavlink_tx", mavlink_lora_msg, queue_size=0)
        self.mission_upload_pub     = rospy.Publisher("/mavlink_interface/mission/mavlink_upload_mission", mavlink_lora_mission_list, queue_size=0)

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
                self.send_mavlink_mission_req(self.mission_id_next)
            else:
                self.busy = False

        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_ACK:
            self.busy = False

        elif msg.msg_id == MAVLINK_MSG_ID_MISSION_CURRENT:
            self.active_mission_item = self.mi.unpack_mission_current(msg.payload)

    def on_mission_list(self, msg):
        self.mission_list_up.waypoints = msg.waypoints

    def on_mission_ack(self, msg):
        self.busy = False
        rospy.loginfo(msg.data)

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
            return TriggerResponse(False, "Can't upload mission. Handler is busy.")
        else:
            self.busy = True
            self.mission_list_up.header.stamp = rospy.Time.now()
            self.mission_upload_pub.publish(self.mission_list_up)
            return TriggerResponse(True, "Uploading mission.")

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
            self.busy = True
            mission_list.header.stamp = rospy.Time.now()
            self.mission_upload_pub.publish(mission_list)
            return UploadFromFileResponse(True, "Uploading mission.")


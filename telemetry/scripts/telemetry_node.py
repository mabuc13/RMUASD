#!/usr/bin/env python3

import signal
import sys
import rospy
import time

from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos, mavlink_lora_status
from gcs.msg import *
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerResponse

# defines
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_LONG_LEN = 33
MAVLINK_MSG_ID_COMMAND_ACK = 77

MAVLINK_CMD_ID_ARM_DISARM = 400
MAVLINK_CMD_ID_MISSION_START = 300
MAVLINK_CMD_NAV_RETURN_TO_LAUNCH = 20
MAVLINK_CMD_NAV_LAND = 21
MAVLINK_CMD_NAV_TAKEOFF = 22
MAVLINK_CMD_NAV_LAND_LOCAL = 23
MAVLINK_CMD_NAV_TAKEOFF_LOCAL = 24

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
mavlink_lora_keypress_sub_topic = '/keypress'
update_interval = 10



class Telemetry(object):

    def __init__(self):
        self.request_sent = False

		# status variables
        self.batt_volt = 0.0
        self.last_heard = 0
        self.last_heard_sys_status = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        # Service handlers
        self.arm_service            = rospy.Service("/telemetry/arm_drone", Trigger, self.arm_drone, buff_size=10)
        self.disarm_service         = rospy.Service("/telemetry/disarm_drone", Trigger, self.disarm_drone, buff_size=10)
        self.takeoff_service        = rospy.Service("/telemetry/takeoff_drone", Trigger, self.takeoff_drone, buff_size=10)
        self.land_service           = rospy.Service("/telemetry/land_drone", Trigger, self.land_drone, buff_size=10)
        self.start_mission_service  = rospy.Service("/telemetry/start_mission", Trigger, self.start_mission, buff_size=10)
        self.return_home_service    = rospy.Service("/telemetry/return_home", Trigger, self.return_home, buff_size=10)

        # Topic handlers
        self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
        rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)
        rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, self.on_mavlink_lora_status)
        rospy.Subscriber(mavlink_lora_keypress_sub_topic, Int8, self.on_keypress)
        self.rate = rospy.Rate(update_interval)
        rospy.sleep (1) # wait until everything is running


    def on_mavlink_msg(self,msg):
        '''
        if msg.msg_id == MAVLINK_MSG_ID_PARAM_VALUE:
            (param_value, param_count, param_index, param_id, param_type) = struct.unpack('<fHH16sB', msg.payload)	
            print param_id, param_value, param_count
        '''
        pass

    def on_mavlink_lora_pos(self,msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

    def on_mavlink_lora_status (self, msg):
        self.last_heard = msg.last_heard.secs + msg.last_heard.nsecs/1.0e9
        self.last_heard_sys_status = msg.last_heard_sys_status.secs + msg.last_heard_sys_status.nsecs/1.0e9
        self.batt_volt = msg.batt_volt / 1000.0

    def on_keypress(self):
        pass

    def send_mavlink_msg_id_cmd_long(self,params,command,id):
        msg = mavlink_lora_msg()

        # no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
        msg.header.stamp = rospy.Time.now()
        msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        msg.sys_id = 0
        msg.comp_id = 0

        # Appears to need to be 1,1
        target_sys = 1
        target_comp = 1

        # command = MAVLINK_CMD_ID_ARM_DISARM
        confirmation = 0

        msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        msg.payload = struct.pack('<7fHBBB', params[0], params[1], params[2], params[3], params[4], params[5], params[6], command, target_sys, target_comp, confirmation)
        self.mavlink_msg_pub.publish(msg)

    # TODO Create suitable service messages for these calls (maybe include drone ID)
    def arm_drone(self, msg):
        command = MAVLINK_CMD_ID_ARM_DISARM
        params = (1,0,0,0,0,0,0)

        self.send_mavlink_msg_id_cmd_long(params,command,1)

        # TODO Handle responses properly
        return True, "Dummy message"

    def disarm_drone(self, msg):
        command = MAVLINK_CMD_ID_ARM_DISARM
        params = (0,0,0,0,0,0,0)

        self.send_mavlink_msg_id_cmd_long(params,command,1)

        # TODO Handle responses properly
        return True, "Dummy message"

    def takeoff_drone(self, msg):
        command = MAVLINK_CMD_NAV_TAKEOFF
        params = (0,0,0,0,self.lat,self.lon,self.alt+5)

        self.send_mavlink_msg_id_cmd_long(params,command,1)

        # TODO Handle responses properly
        return True, "Dummy message"

    def land_drone(self, msg):
        command = MAVLINK_CMD_NAV_LAND
        params = (0,0,0,0,self.lat,self.lon,0)

        self.send_mavlink_msg_id_cmd_long(params,command,1)

        # TODO Handle responses properly
        return True, "Dummy message"

    def start_mission(self, msg):
        command = MAVLINK_CMD_ID_MISSION_START
        params = (0,0,0,0,0,0,0)

        self.send_mavlink_msg_id_cmd_long(params,command,1)

        # TODO Handle responses properly
        return True, "Dummy message"

    def return_home(self,msg):
        command = MAVLINK_CMD_NAV_RETURN_TO_LAUNCH
        params = (0,0,0,0,0,0,0)

        self.send_mavlink_msg_id_cmd_long(params,command,1)

        # TODO Handle responses properly
        return True, "Dummy message"

    def shutdownHandler(self):
        # shutdown services
        self.arm_service.shutdown()
        self.disarm_service.shutdown()
        print("Shutting down")


def main():
    rospy.init_node('telemetry')#, anonymous=True)
    rospy.sleep(1)

    tel = Telemetry()

    rospy.on_shutdown(tel.shutdownHandler)

    rospy.spin()

if __name__ == "__main__":
    main()

'''
def signal_handler(signal,somthing):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def getDroneStatus():
    msg = DroneInfo();
    msg.position.longitude  = 55.6
    msg.position.latitude   = 55.6
    msg.position.altitude   = 100
    msg.next_goal.longitude = 57.6
    msg.next_goal.latitude  = 57.6
    msg.next_goal.altitude  = 100
    msg.velocity[0] = 10.0
    msg.velocity[1] = 1
    msg.velocity[2] = 0.1
    msg.heading = 360 #deg
    msg.battery_SOC = 68 #Percent
    msg.drone_id = 2524362
    msg.GPS_timestamp = 1251351312
    msg.status = 0
    return msg

def new_route_reqest_handler(msg):
    pass

def Drone_State_request_handler(msg):
    pass

DroneStatus_pub = rospy.Publisher('/Telemetry/DroneStatus',DroneInfo, queue_size=10)
RouteRequest_sub = rospy.Subscriber('/gcs/PathRequest', DronePath, new_route_reqest_handler)
DroneState_sub = rospy.Subscriber('/gcs/StateRequest', DroneState, Drone_State_request_handler)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('DroneLinkNode')
    rate = 50
    spins = -1
    while not rospy.is_shutdown():
        spins = spins +1
        rospy.Rate(rate).sleep()

        if spins % rate == 0:
            DroneStatus_pub.publish(getDroneStatus())
'''
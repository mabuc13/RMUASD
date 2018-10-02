#!/usr/bin/env python3

import signal
import sys
import rospy
import time

from gcs.msg import *

class Telemetry(object):

    def __init__(self):
        mavlink_lora_sub_topic = '/mavlink_rx'
        mavlink_lora_pub_topic = '/mavlink_tx'

        rospy.init_node('mavlink_lora_gcs_simple', disable_signals = True)
		self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
		rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, self.on_mavlink_lora_status)
		rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)
		rospy.Subscriber(mavlink_lora_atti_sub_topic, mavlink_lora_attitude, self.on_mavlink_lora_attitude)
		rospy.Subscriber(mavlink_lora_keypress_sub_topic, Int8, self.on_keypress)
		self.rate = rospy.Rate(update_interval)
		rospy.sleep (1) # wait until everything is running


    def on_mavlink_msg(self, msg):
        pass

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
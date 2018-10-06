#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) param list example script
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
This example script shows how to set mode on a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
'''

# parameters
mavlink_lora_sub_topic = '/mavlink_interface/command/ack'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_set_mode_pub_topic = '/mavlink_interface/command/set_mode'

# defines


# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg
from mavlink_lora.msg import mavlink_lora_command_set_mode, mavlink_lora_command_ack
from std_msgs.msg import Bool

# variables
target_sys = 0 # reset by first message
target_comp = 0
home_lat = 55.057883
home_lon = 10.569678

def on_mavlink_msg (msg):
    print(msg)


def send_mavlink_set_mode(mode, custom_mode, custom_sub_mode):
    msg = mavlink_lora_command_set_mode()
    msg.mode = mode
    msg.custom_mode = custom_mode
    msg.custom_sub_mode = custom_sub_mode
    mavlink_set_mode_pub.publish(msg)


# launch node
rospy.init_node('mavlink_lora_set_mode')
mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0) # mavlink_msg publisher

# ack sub
rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_command_ack, on_mavlink_msg) # mavlink_msg subscriber

mavlink_set_mode_pub = rospy.Publisher(mavlink_lora_set_mode_pub_topic, mavlink_lora_command_set_mode, queue_size=0)

rospy.sleep(1) # wait until everything is running

# MAV_MODE_PREFLIGHT 0
# MAV_MODE_STABILIZE_DISARMED 80
# MAV_MODE_STABILIZE_ARMED 208
# MAV_MODE_MANUAL_DISARMED 64
# MAV_MODE_MANUAL_ARMED 192
# MAV_MODE_GUIDED_DISARMED 88
# MAV_MODE_GUIDED_ARMED 216
# MAV_MODE_AUTO_DISARMED 92
# MAV_MODE_AUTO_ARMED 220
# MAV_MODE_TEST_DISARMED 66
# MAV_MODE_TEST_ARMED 194


print("set_mode preflight: 80")
send_mavlink_set_mode(80, 0, 0)

rospy.sleep(10)

print("set_mode auto_disarmed: 92")
send_mavlink_set_mode(92, 0, 0)


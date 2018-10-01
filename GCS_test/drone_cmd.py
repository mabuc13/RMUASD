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
This example script shows how to arm a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
'''
# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
update_interval = 0.2

# defines
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_LONG_LEN = 33
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_GPS_RTCM_DATA = 233
MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN = 182

MAVLINK_CMD_ID_ARM_DISARM = 400
MAVLINK_CMD_ID_MISSION_START = 300
MAVLINK_CMD_NAV_RETURN_TO_LAUNCH = 20
MAVLINK_CMD_NAV_LAND = 21
MAVLINK_CMD_NAV_TAKEOFF = 22
MAVLINK_CMD_NAV_LAND_LOCAL = 23
MAVLINK_CMD_NAV_TAKEOFF_LOCAL = 24

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_msg

# variables
msg = mavlink_lora_msg()
request_sent = False
first_msg_ok = False
target_sys = 0 # reset by first message
target_comp = 0
home_lat = 55.057883
home_lon = 10.569678

def on_mavlink_msg (msg):
	global first_msg_ok
	global target_sys
	if first_msg_ok == False:
		first_msg_ok = True
		target_sys = msg.sys_id

	if msg.msg_id == MAVLINK_MSG_ID_COMMAND_ACK:
		(command, result) = struct.unpack('<HB', msg.payload)	
		print command, result

def send_mavlink_arm(mavlink_pub):
	# no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
	msg.header.stamp = rospy.Time.now()
	msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
	msg.sys_id = 0
	msg.comp_id = 0

	# Payload
	param1 = 1
	param2 = 0
	param3 = 0
	param4 = 0
	param5 = 0
	param6 = 0
	param7 = 0

	# Appears to need to be 1,1
	target_sys = 1
	target_comp = 1

	command = MAVLINK_CMD_ID_ARM_DISARM
	confirmation = 0

	msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
	msg.payload = struct.pack('<7fHBBB', param1, param2, param3, param4, param5, param6, param7, command, target_sys, target_comp, confirmation)
	mavlink_pub.publish(msg)

def send_mavlink_takeoff(mavlink_pub, lat, lon, alt):
	msg.header.stamp = rospy.Time.now()
	msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
	msg.sys_id = 0
	msg.comp_id = 0

	# Payload
	param1 = 0
	param2 = 0
	param3 = 0
	param4 = 45
	param5 = lat
	param6 = lon
	param7 = alt + 5

	# Appears to need to be 1,1
	target_sys = 1
	target_comp = 1

	command = MAVLINK_CMD_NAV_TAKEOFF
	confirmation = 0

	msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
	msg.payload = struct.pack('<7fHBBB', param1, param2, param3, param4, param5, param6, param7, command, target_sys, target_comp, confirmation)
	mavlink_pub.publish(msg)

def send_mavlink_land(mavlink_pub, lat, lon):
	msg.header.stamp = rospy.Time.now()
	msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
	msg.sys_id = 0
	msg.comp_id = 0

	# Payload
	param1 = 0
	param2 = 0
	param3 = 0
	param4 = 0
	param5 = lat
	param6 = lon
	param7 = 0

	# Appears to need to be 1,1
	target_sys = 1
	target_comp = 1

	command = MAVLINK_CMD_NAV_LAND
	confirmation = 0

	msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
	msg.payload = struct.pack('<7fHBBB', param1, param2, param3, param4, param5, param6, param7, command, target_sys, target_comp, confirmation)
	mavlink_pub.publish(msg)

def send_mavlink_return(mavlink_pub):
	msg.header.stamp = rospy.Time.now()
	msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
	msg.sys_id = 0
	msg.comp_id = 0

	# Payload
	param1 = 0
	param2 = 0
	param3 = 0
	param4 = 0
	param5 = 0
	param6 = 0
	param7 = 0

	# Appears to need to be 1,1
	target_sys = 1
	target_comp = 1

	command = MAVLINK_CMD_NAV_RETURN_TO_LAUNCH
	confirmation = 0

	msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
	msg.payload = struct.pack('<7fHBBB', param1, param2, param3, param4, param5, param6, param7, command, target_sys, target_comp, confirmation)
	mavlink_pub.publish(msg)

def send_mavlink_start_mission(mavlink_pub):
	msg.header.stamp = rospy.Time.now()
	msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
	msg.sys_id = 0
	msg.comp_id = 0

	# Payload
	param1 = 0
	param2 = 0
	param3 = 0
	param4 = 0
	param5 = 0
	param6 = 0
	param7 = 0

	# Appears to need to be 1,1
	target_sys = 1
	target_comp = 1

	command = MAVLINK_CMD_ID_MISSION_START
	confirmation = 0

	msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
	msg.payload = struct.pack('<7fHBBB', param1, param2, param3, param4, param5, param6, param7, command, target_sys, target_comp, confirmation)
	mavlink_pub.publish(msg)


if __name__ == "__main__":
	# launch node
	rospy.init_node('mavlink_lora_arm_disarm')
	mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0) # mavlink_msg publisher
	rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg) # mavlink_msg subscriber
	rate = rospy.Rate(update_interval)
	rospy.sleep (1) # wait until everything is running

	# loop until shutdown
	takeoff = False
	while not (rospy.is_shutdown()):

		# do stuff
		if takeoff == True:
			print "Sending takeoff command"
			#send_mavlink_takeoff()
			send_mavlink_takeoff()
			takeoff = False

		if request_sent == False and first_msg_ok == True:
			print 'Arming'
			send_mavlink_arm()
			request_sent = True	
			takeoff = True
		
		rospy.sleep(5)

		# sleep the defined interval
		#rate.sleep()


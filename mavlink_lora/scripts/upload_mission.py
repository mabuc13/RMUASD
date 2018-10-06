#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) upload mission example script
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
This example script shows how to upload a mission to a drone.

It has been tested using a Pixhawk 2.1 flight controller running PX4 and on
an AutoQuad flight controller. Please remember to set the correct
target_system value below.

Revision
2018-06-13 FMA First published version
'''
# parameters
mavlink_lora_pub_topic = 'mavlink_interface/mission/mavlink_upload_mission'

# defines

# imports
import rospy
import struct
from mavlink_lora.msg import mavlink_lora_mission_list
from mavlink_lora.msg import mavlink_lora_mission_item_int

# variables
target_sys = 0 # reset by first message
target_comp = 0
home_lat = 55.4720252
home_lon = 10.4146084

# launch node
rospy.init_node('mavlink_lora_mission_upload')
mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_mission_list, queue_size=0) # mavlink_msg publisher

rospy.sleep (1) # wait until everything is running

# make 4-5 waypoints and save in list.
missionlist = mavlink_lora_mission_list()

# CHANGE SPEED
speed = mavlink_lora_mission_item_int()
speed.target_system = 0
speed.target_component = 0
speed.seq = 0
speed.frame = 2 # mission command frame
speed.command = 178 # set home
speed.param1 = 0 # air_speed
speed.param2 = 5 # m/s
speed.param3 = -1 # no change
speed.param4 = 0 # abosulte or relative. relative = 1

# TAKEOFF waypoint
way1 = mavlink_lora_mission_item_int()
way1.target_system = 0
way1.target_component = 0
way1.seq = 1
way1.frame = 6 #global pos, relative alt_int
way1.command = 22
way1.x = home_lat * 10000000
way1.y = home_lon * 10000000
way1.z = 20
way1.current = 1

# WAYPOINT 1
way2 = mavlink_lora_mission_item_int()
way2.target_system = 0
way2.target_component = 0
way2.seq = 2
way2.frame = 6 #global pos, relative alt_int
way2.command = 16
way2.param1 = 0 # hold time
way2.param2 = 5 # acceptance radius in m
way2.param3 = 0 # pass though waypoint, no trajectory control
way2.x = 55.4720010 * 10000000
way2.y = 10.4164463 * 10000000
way2.z = 20

# WAYPOINT 2
way3 = mavlink_lora_mission_item_int()
way3.target_system = 0
way3.target_component = 0
way3.seq = 3
way3.frame = 6 #global pos, relative alt_int
way3.command = 16
way3.param1 = 0 # hold time
way3.param2 = 5 # acceptance radius in m
way3.param3 = 0 # pass though waypoint, no trajectory control
way3.x = 55.4720615 * 10000000
way3.y = 10.4161885 * 10000000
way3.z = 40

# WAYPOINT 3
way4 = mavlink_lora_mission_item_int()
way4.target_system = 0
way4.target_component = 0
way4.seq = 4
way4.frame = 6 #global pos, relative alt_int
way4.command = 16
way4.param1 = 0 # hold time
way4.param2 = 5 # acceptance radius in m
way4.param3 = 0 # pass though waypoint, no trajectory control
way4.x = 55.4721370 * 10000000
way4.y = 10.4164410 * 10000000
way4.z = 30

# WAYPOINT 4 LANDING
way5 = mavlink_lora_mission_item_int()
way5.target_system = 0
way5.target_component = 0
way5.seq = 5
way5.frame = 6 #global pos, relative alt_int
way5.command = 21
way5.param1 = 5 # abort alt
way5.param2 = 0 # precision landing. 0 = normal landing
way5.x = home_lat * 10000000
way5.y = home_lon * 10000000
way5.z = 20

# add waypoints to list
missionlist.waypoints.append(speed)
missionlist.waypoints.append(way1)
missionlist.waypoints.append(way2)
missionlist.waypoints.append(way3)
missionlist.waypoints.append(way4)
missionlist.waypoints.append(way5)

mavlink_msg_pub.publish(missionlist)

# loop until shutdown
while not (rospy.is_shutdown()):

	# do stuff
	rospy.sleep(5)


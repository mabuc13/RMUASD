#!/usr/bin/env python
#/***************************************************************************
# MavLink LoRa node (ROS) mission download script
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
# (INCLUDING NEGIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
Revision
2018-06-12 KJ First published version
'''
# parameters
target_sys = 92 # target system (1 = Pixhawk2 PX4, 66 = AutoQuad)
target_comp = 0 # target component

# other defines
ros_node_name = 'mission_download'
ros_node_update_interval = 10
mavlink_lora_rx_sub = '/mavlink_rx'
mavlink_lora_tx_pub = '/mavlink_tx'

# imports
from sys import argv
import rospy
from mission_lib import * # pylint: disable=W0614
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list

# defines
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

MAX_RETRIES = 10
TIMEOUT = 1.5

class ros_node():
	def __init__(self, target_sys, target_comp):
		# initiate variables
		self.stop = False
		self.first_msg_ok = False
		self.request_sent = False
		self.mission_count = 0
		self.mission_id_next = 0
		self.mi = mission_lib()
		self.sys_id = 0 # reset when receiving first msg
		self.comp_id = 0
		self.mission_list = mavlink_lora_mission_list()	

		self.download_timer = None
		self.retries = 0

		# initiate node
		rospy.init_node(ros_node_name)
		self.mission_list_pub = rospy.Publisher("/telemetry/new_mission", mavlink_lora_mission_list, queue_size=0)
		self.mavlink_msg_pub = rospy.Publisher(mavlink_lora_tx_pub, mavlink_lora_msg, queue_size=0)
		rospy.Subscriber(mavlink_lora_rx_sub, mavlink_lora_msg, self.on_mavlink_msg)
		self.rate = rospy.Rate(ros_node_update_interval)

		# wait until everything is running (important)
		rospy.sleep (1)

	def on_mavlink_msg (self, msg):
		if self.first_msg_ok == False:
			self.first_msg_ok = True
			self.sys_id = msg.sys_id
			self.mi.set_target(self.sys_id, self.comp_id)

		if msg.msg_id == MAVLINK_MSG_ID_MISSION_ITEM:
			# stop the timeout
			self.download_timer.shutdown()
			# add the item to the list
			mission_item = self.mi.unpack_mission_item(msg.payload)
			print(mission_item)
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

				self.mission_list.waypoints.append(mission_item_msg)

			if self.mission_id_next < self.mission_count - 1:
				self.mission_id_next += 1
				# if self.mission_id_next == 10:
				# 	self.mission_id_next += 2
				print("Requesting item #%d" % self.mission_id_next)
				self.send_mavlink_mission_req(self.mission_id_next)
			else:
				self.mission_list.header.stamp = rospy.Time.now()
				self.mission_list_pub.publish(self.mission_list)
				self.stop = True
				
		elif msg.msg_id == MAVLINK_MSG_ID_MISSION_COUNT:
			# stop the timeout
			self.download_timer.shutdown()

			self.mission_count = self.mi.unpack_mission_count(msg.payload)
			print("Mission count: %d" % self.mission_count)
			if self.mission_count > 0:
				self.mission_id_next = 0
				self.send_mavlink_mission_req(self.mission_id_next)
			else:
				self.stop = True

	def send_mavlink_mission_req_list(self):
		self.mi.msg.header.stamp = rospy.Time.now()
		self.mi.pack_mission_req_list()
		self.mavlink_msg_pub.publish(self.mi.msg)

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
			rospy.loginfo("Too many failures - shutting down")
			self.stop = True
		else:
			rospy.loginfo("Requesting list again - %d. retry" % self.retries)
			self.send_mavlink_mission_req_list()

	def request_item_timeout(self, event):
		self.retries += 1		
		rospy.loginfo("Item request timeout!")

		if self.retries > MAX_RETRIES:
			self.retries = 0
			rospy.loginfo("Too many failures - shutting down")
			self.stop = True
		else:
			rospy.loginfo("Requesting item #%d again" % self.mission_id_next)
			self.send_mavlink_mission_req(self.mission_id_next)

	def loop(self):
		while not (rospy.is_shutdown() or self.stop):
			# do stuff
			if self.request_sent == False and self.first_msg_ok == True:
				print("Requesting mission list")
				self.send_mavlink_mission_req_list()
				self.request_sent = True	

			# sleep the defined interval
			self.rate.sleep()

if __name__ == "__main__":
	rn = ros_node(target_sys, target_comp)
	rn.loop()


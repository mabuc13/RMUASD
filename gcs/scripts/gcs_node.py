#!/usr/bin/env python3
import rospy
import numpy as np
import os

#
import queue
from stringMsgReader import *
from drone import *
from dockingstation import *
from job import *
from gcs.msg import *
from std_msgs.msg import String

# List of own drones
Drones = []
dockingStations = []
Jobs_q = queue.queue()

def String2Dock(text):
	if isinstance(text,str):
		text = csvText(text)

	if not text.valueOf("name") ==  "NULL" and 		\
	   not text.valueOf("longitude") == "NULL" and 	\
	   not text.valueOf("latitude") == "NULL" and 	\
	   not text.valueOf("altitude") == "NULL":
	####
		dockingStations.append( 				\
			dockingstation(		\
			text.valueOf("name"), 				\
			text.valueOf("longitude"), 			\
			text.valueOf("latitude"), 			\
			text.valueOf("altitude"), 			\
			text.valueOf("lab")					\
			))
		####
		return True
	else:
		return False

def Web_handler(msg):
	msg = csvText(str(msg.data))
	if not msg.valueOf("name") == "NULL":
		feedBack = String()
		feedBack = "name=gcs,target="+msg.valueOf(name)
		if msg.valueOf("register").lower() == "true":
			if String2Dock(msg):
				print("DockingStation Registered: " + dockingStations[-1].get_name())
				feedBack = feedBack + ",register=succes"
			else:
				feedBack = feedBack + ",register=failed"

		elif msg.valueOf("request").lower() == "true":
			Dock = None
			for dock in dockingStations:
				if dock.get_name() == msg.valueOf("name"):
					Dock = dock
					continue
			if not Dock == None:
				aJob = job(Dock)
				Jobs_q.put(aJob)
				feedBack = feedBack + ",request=queued"
			elif   not msg.valueOf("latitude") and \
				   not msg.valueOf("longtitude") and \
				   not msg.valueOf("altitude"):
				####
				aDock = dockingstation(msg.valueOf("name"),msg.valueOf("latitude"),msg.valueOf("longitude"),msg.valueOf("altitude"),False)
				aJob = job(aDock)
				Jobs_q.put(aJob)



		WebInfo_pub.publish(feedBack)
	print(msg)

def DroneStatus_handler(msg):
	new_drone = True
	for d in Drones:
		if msg.drone_id == d.get_ID():
			new_drone = False
			continue
	if new_drone:
		Drones.append(drone(msg.drone_id,msg.position))
		print("Drone Registered: " + str(Drones[len(Drones)-1].get_ID()))

DroneStatus_sub = rospy.Subscriber('/Telemetry/DroneStatus',DroneInfo, DroneStatus_handler)
RouteRequest_pub = rospy.Publisher('/gcs/PathRequest', DronePath, queue_size=10)
DroneState_pub = rospy.Publisher('/gcs/StateRequest', DroneState, queue_size=10)
WebInfo_sub = rospy.Subscriber('/FromInternet',String, Web_handler)
WebInfo_pub = rospy.Publisher('/ToInternet', String, queue_size = 10)

def initialize():
	global no_docks, no_drones
	rospy.init_node('GroundControlStation')
	# Create two dockingstations with their location:
	path = __file__[:len(__file__)-11]+"/Settings/DockingStationsList.txt"
	with open(path) as f:
		content = f.readlines()
		for line in content:
			if String2Dock(line):
				print("DockingStation Registered: " + dockingStations[-1].get_name())
			else:
				print("DockingStation Register Failed")

if __name__ == "__main__":
	initialize()

	while not rospy.is_shutdown():

		# Check new information from UTM server:

			#global_map.append(new_info)
			#If new information, does current drones need to be redirected?

		# Check for new input from user:

			# If new input, find free drone:
			for d in Drones:
				if d.isAvailable:
					# Select this one.
					pass
				else:
					continue

			# Add start and goal docking:

			# Compute path:

			# Add path

			# Start drone

		# Go through list of drones, and update ETA to UI

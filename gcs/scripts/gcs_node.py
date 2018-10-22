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
Jobs_q = queue.Queue()
active_Jobs = []
def webMSG(target,msg):
	text = "name=gcs,target="+target+","+msg
	WebInfo_pub.publish(text)

def ETAofJob(job):
	return 42

def getDrone(ID):
	for d in Drones:
		if d.get_ID() == ID:
			return d
def getJob(ID):
	for j in active_Jobs:
		if j.drone == ID:
			return j

def NearestLab(position):
	for Dock in dockingStations:
		if Dock.isLab:
			return Dock
	return None

def PathPlaner(position1, position2):
	return [position1,position2]

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
		if msg.valueOf("request").lower() == "true":
			Dock = None
			for dock in dockingStations:
				if dock.get_name() == msg.valueOf("name"):
					Dock = dock
					break
			if not Dock == None:
				aJob = job(Dock)
				Jobs_q.put(aJob)
				feedBack = feedBack + ",request=queued"
			elif   not msg.valueOf("latitude") and \
				   not msg.valueOf("longtitude") and \
				   not msg.valueOf("altitude"):
				####
				dockingStations.append(dockingstation(msg.valueOf("name"),		\
												      msg.valueOf("latitude"),	\
													  msg.valueOf("longitude"),	\
													  msg.valueOf("altitude"),	\
													  False))
				Jobs_q.put(job(dockingStations[-1]))
				feedBack = feedBack + ",request=queued,register=succes"
		if not msg.valueOf("return") == "NULL":
			currentJob = None
			for job in active_Jobs:
				if job.status == job.onhold and job.dock.get_name() == msg.valueOf("name"):
					currentJob = job
			if not currentJob == None:
				active_Jobs.remove(currentJob)
				ret = msg.valueOf("return")
				for Dock in dockingStations:
					if Dock.get_name() == ret:
						ret = Dock
						break
				if isinstance(ret,str):
					currentJob.dock = NearestLab(currentJob.dock.get_location())
				else:
					currentJob.dock = ret
				currentJob.status= currentJob.wait4pathplan
				active_Jobs.append(currentJob)
				feedBack = feedBack + ",request=queued"
			else:
				feedBack = feedBack + ",request=failed"

		WebInfo_pub.publish(feedBack)
	print(msg)

def DroneStatus_handler(msg):
	global Drones
	new_drone = True
	for d in Drones:
		if msg.drone_id == d.get_ID():
			new_drone = False
			break
	if new_drone:
		Drones.append(drone(msg.drone_id,msg.position))
		print("Drone Registered: " + str(Drones[len(Drones)-1].get_ID()))
	else:
		i = Drones.index(getDrone(msg.drone_id))
		Drones[i].update(msg)
	if msg.status == msg.Run:
		job = getJob(msg.drone_id)
		if not job.status == job.ongoing:
			i = active_Jobs.index(job)
			active_Jobs[i].status = job.ongoing

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
		if not Jobs_q.empty():
			# If new input, find free drone:
			for d in Drones:
				if d.isAvailable():
					i = Drones.index(d)
					aJob = Jobs_q.get_nowait()
					Drone[i].set_Job(aJob)
					aJob.attachDrone(d)
					active_Jobs.append(aJob)
					webMSG(aJob.requestGiver,"request=pathplaning")

			# Compute path:
		for job in active_Jobs:
			if job.status == job.wait4pathplan:
				drone = getDrone(job.drone)
				path = PathPlaner(drone.get_Position(),job.dock.get_location())
				i = active_Jobs.index(job)
				l = Drones.index(drone)
				Drones[i].set_path(path)
				msg = DronePath()
				msg.Path = path
				msg.DroneID = drone.get_ID
				RouteRequest_pub.publish(msg)
				active_Jobs[i].status = job.ready4takeOff
				webMSG(job.requestGiver,"request=ready for takeoff")

		for job in active_Jobs:
			if job.status == job.ongoing:
				ETA = ETAofJob(job)
				webMSG(job.requestGiver,"request=ongoing,ETA="+str(ETA))

		# Go through list of drones, and update ETA to UI

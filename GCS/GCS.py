#!/usr/bin/env python3
import rospy
import numpy as np

#
import drone
import dockingstation
from gcs.msg import *

# List of own drones
list_drones = []
list_docks = []

# Global variables:
setup = False
global_map = []
no_docks = 0
no_drones = 0

def DroneStatus_handler(msg):
	print(msg)

DroneStatus_sub = rospy.Subscriber('/Telemetry/DroneStatus',DroneInfo, DroneStatus_handler)
RouteRequest_pub = rospy.Publisher('/gcs/PathRequest', DronePath, queue_size=1)
DroneState_pub = rospy.Publisher('/gcs/StateRequest', DroneState, queue_size=1)


def initialize():
	global no_docks, no_drones
	rospy.init_node('GroundControlStation')
	# Create two dockingstations with their location:
	dock_0 = dockingstation.dockingstation(no_docks, 55.0, 10.0, 0)
	no_docks = no_docks +1
	dock_1 = dockingstation.dockingstation(no_docks, 55.0, 10.0, 0)
	no_docks = no_docks+ 1

	# Create the drones available:
	drone_0 = drone.drone(no_drones)
	no_drones = no_drones+ 1

	# Add to lists:
	list_docks.append(dock_0)
	list_docks.append(dock_1)
	list_drones.append(drone_0)

	setup = True

if __name__ == "__main__":

	# Run setup rutine if this is startup:
	if not setup:
		initialize()

	while not rospy.is_shutdown():

		# Check new information from UTM server:

			#global_map.append(new_info)
			# If new information, does current drones need to be redirected?

		# Check for new input from user:

			# If new input, find free drone:
			for d in list_drones:
				if d.get_status == 0:
					# Select this one.
					pass
				else:
					continue

			# Add start and goal docking:

			# Compute path:

			# Add path

			# Start drone

		# Go through list of drones, and update ETA to UI

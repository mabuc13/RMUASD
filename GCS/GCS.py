#!/usr/bin/python

import numpy as np

#
import drone
import dockingstation

# List of own drones
list_drones = np()
list_docks = np()

# Global variables:
setup = false
global_map = np()
no_docks = 0
no_drones = 0

def initialize:

	# Create two dockingstations with their location:
	dock_0 = dockingstation(no_docks, 55.0, 10.0, 0)
	no_docks += 1
	dock_1 = dockingstation(no_docks, 55.0, 10.0, 0)
	no_docks += 1

	# Create the drones available:
	drone_0 = drone(no_drones)
	no_drones += 1

	# Add to lists:
	list_docks.append(dock_0)
	list_docks.append(dock_1)
	list_drones.append(drone_0)

	setup = true

if __name__ = "__main__":

	# Run setup rutine if this is startup:
	if not setup:
		initialize()

	while true:

		# Check new information from UTM server:

			global_map.append(new_info)
			# If new information, does current drones need to be redirected?

		# Check for new input from user:

			# If new input, find free drone:
			for d in list_drones:
				if d.get_status == 0:
					# Select this one.
				else:
					continue

			# Add start and goal docking:

			# Compute path:

			# Add path

			# Start drone

		# Go through list of drones, and update ETA to UI


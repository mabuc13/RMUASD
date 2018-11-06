#!/usr/bin/env python


from coordinate import Coordinate
import numpy as np
import rospy
from gcs.msg import DroneInfo, GPS, DronePath
from math import sqrt, cos, sin


class CollisionDetector:
    '''
    Subscribe to "/gcs/forwardPath". It sends out the whole pathplan from the GCS to the drone.
    Subscribe to "/drone_handler/DroneInfo". It contains info on the specific drones position etc.
    '''
    def __init__(self):
        self.active_drone_paths = {}
        self.active_drone_position_index = {}

        # status variables
        rospy.sleep(1)  # wait until everything is running

        self.collision_detected_pub = rospy.Publisher("/collision_detector/collision_ahead", DroneInfo, queue_size=0)

        # Subscribe to new drone path's, and dynamic obstacles:
        rospy.Subscriber("/gcs/forwardPath", DronePath, self.on_drone_path)
        rospy.Subscriber("/drone_handler/DroneInfo", DroneInfo, self.on_drone_info)

    def on_drone_path(self, msg):
        self.active_drone_paths[msg.DroneID] = msg.Path

    def on_drone_info(self, msg):
        # Adding the next waypoint index in the plan to the dict
        if msg.status == msg.Run:
            self.active_drone_position_index[msg.DroneID] = msg.mission_index
        if msg.

    def check_for_collisions(self):
        for drone_id, path in self.active_drone_paths:
            collision, p1, p2 = self.check_path_for_collision(path, drone_id)
            if (collision)
                pass
                # Collision

    def check_path_for_collision(self, path, drone_id):
        # Create to index's to iterate through the path plan. "i" is the first waypoint, "j" is the second one.
        # If "i" is larger than "0", then set it to the index of the previously passed waypoint in the plan. There is
        # no need to check for collisions on the path, in places where the drone has already passed and is not coming
        # back to.
        i = 0
        if self.active_drone_position_index[drone_id] > 0:
            i = self.active_drone_position_index[drone_id] - 1
        j = i + 1
        while j <= len(path):
            p1 = Coordinate(lat=path[i].latitude, lon=path[i].longitude)
            p2 = Coordinate(lat=path[j].latitude, lon=path[j].longitude)
            p3 = p1
            dist = self.pythagoras(p1, p2)
            delta_x = p2.easting - p1.easting
            delta_y = p2.northing - p1.northing

            # Move in a straight line between
            for k in range(1, int(dist)):
                p3.easting = p1.easting + delta_x * float(k/dist)
                p3.northing = p1.northing + delta_y * float(k/dist)
                #p3.update_geo_coordinates()

                # Check for collision:
                # if (coordinate == no-fly-zone):
                #     # Collision. Return true, and the two points the collision is in between.
                #     return True, p1, p2
            i += 1
            j += 1
        return False, None, None

    def pythagoras(self, p1, p2):
        return sqrt((p1.easting - p2.easting)**2 + (p1.northing - p2.northing)**2)


if __name__ == "__main__":
    rospy.init_node('collision_detector')  # , anonymous=True)
    rospy.sleep(1)

    cd = CollisionDetector()


#!/usr/bin/env python


from coordinate import Coordinate
import rospy
from gcs.msg import DroneInfo, GPS, DronePath
from math import sqrt
import time
import json
import string
from shapely import geometry


class CollisionDetector:
    '''
    Subscribe to "/gcs/forwardPath". It sends out the whole pathplan from the GCS to the drone.
    Subscribe to "/drone_handler/DroneInfo". It contains info on the specific drones position etc.
    '''
    def __init__(self, safety_dist=10, safety_time=10):     # 10 m and 10 sec as default
        self.active_drone_paths = {}
        self.active_drone_info = {}
        self.dist_between_mission_points = {}
        self.dynamic_no_flight_zones = {}

        self.safety_dist_to_dnfz = safety_dist
        self.safety_extra_time = safety_time

        # status variables
        rospy.sleep(1)  # wait until everything is running

        self.collision_detected_pub = rospy.Publisher("/collision_detector/collision_warning", DroneInfo, queue_size=0)

        # Subscribe to new drone path's, and dynamic obstacles:
        rospy.Subscriber("/gcs/forwardPath", DronePath, self.on_drone_path)
        rospy.Subscriber("/drone_handler/DroneInfo", DroneInfo, self.on_drone_info)
        rospy.Substriber("/utm/dynamic_no_fly_zones", string, self.on_dynamic_no_fly_zones)

    def on_drone_path(self, msg):
        '''
        Callback function
        '''
        self.active_drone_paths[msg.DroneID] = msg.Path
        self.calc_dist_between_mission_points(msg.Path, msg.DroneID)
        collision, p1, p2 = self.check_path_for_collision_with_dnfz(msg.Path, msg.DroneID)
        if collision:
            # There's a collision with dynamic zones between p1 and p2:
            pass

    def on_drone_info(self, msg):
        '''
        Callback function
        '''
        # Adding the next waypoint index in the plan to the dict
        if msg.status == msg.Run:
            self.active_drone_info[msg.DroneID] = msg
        if msg.status == msg.Land:
            # Drone has landed, so delete the path and ID from the dict's
            del self.active_drone_info[msg.DroneID]
            del self.active_drone_paths[msg.DroneID]

    def on_dynamic_no_fly_zones(self, msg):
        '''
        Callback function
        Use "int_id" in the string as key in the dict of dnfz
        '''
        try:
            all_json_objs = json.loads(msg)
            for json_obj in all_json_objs:
                self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
        except ValueError:
            print("Collision Detector: Couldn't convert string from UTM server to json..")

    def run_collision_check(self):
        '''
        This function will go through all active drones, and look into the future positions to check for possible
        collisions with dynamic obstacle.
        '''
        # For all active drones:
        for i, val in self.active_drone_info.items():
            # for 5, 10, 15, ... , 60 seconds into the future:
            for j in range(5,60,5):
                position = self.calc_future_position(val.drone_id,j)
                future_time = time.time() + j
                collision, int_id = self.is_collision_at_future_position(position, future_time)
                if collision:
                    print("Collision detected in future....")
                # Look at dynamic obstacles and see if there's a collision:

    def calc_dist_between_mission_points(self, path, drone_id):
        '''
        This function calculates the distance between each mission point in the drone path. It stores them in a list
        which is then accessible later on. The list is placed in a dict "self.dist_between_mission_points" with
        "DroneID" as key.
        '''
        dists = []
        for i in range(1, len(path)):
            p1 = Coordinate(GPS_data=path[i-1])
            p2 = Coordinate(GPS_data=path[i])
            dists.append(self.pythagoras(p1, p2))
        self.dist_between_mission_points[drone_id] = dists

    def calc_future_position(self, drone_id, sec):
        '''
        This function calculates the future position of the drone, given the current position, the speed, the mission
        and the amount of seconds within the future we are looking. The future position will be estimated based on a
        straight line between mission points.
        '''
        current_position = Coordinate(GPS_data=self.active_drone_info[drone_id].position)
        next_position = Coordinate(GPS_data=self.active_drone_info[drone_id].next_waypoint)

        accumulated_mission_dist = self.pythagoras(current_position, next_position)

        dist_to_travel = self.active_drone_info[drone_id].ground_speed_setpoint * sec

        if accumulated_mission_dist > dist_to_travel:
            # future position is before next mission point
            new_easting  = (dist_to_travel * (next_position.easting - current_position.easting) / accumulated_mission_dist) + current_position.easting
            new_northing = (dist_to_travel * (next_position.northing - current_position.northing) / accumulated_mission_dist) + current_position.northing
            return Coordinate(northing=new_northing, easting=new_easting)
        else:
            for i in range(self.active_drone_info[drone_id].mission_index, len(self.dist_between_mission_points[drone_id])):
                accumulated_mission_dist += self.dist_between_mission_points[drone_id][i]
                if accumulated_mission_dist > dist_to_travel:
                    # future position is before next mission point
                    resulting_dist = dist_to_travel - accumulated_mission_dist + self.dist_between_mission_points[drone_id][i]
                    p1 = Coordinate(lat=self.active_drone_paths[drone_id][i].latitude, lon=self.active_drone_paths[drone_id][i].longitude)
                    p2 = Coordinate(lat=self.active_drone_paths[drone_id][i+1].latitude, lon=self.active_drone_paths[drone_id][i+1].longitude)
                    new_easting = (resulting_dist * (p2.easting - p1.easting) / self.dist_between_mission_points[drone_id][i]) + p1.easting
                    new_northing = (resulting_dist * (p2.northing - p1.northing) / self.dist_between_mission_points[drone_id][i]) + p1.northing
                    return Coordinate(northing=new_northing, easting=new_easting)

    def check_path_for_collision_with_dnfz(self, path, drone_id):
        '''
        Create to index's to iterate through the path plan. "i" is the first waypoint, "j" is the second one.
        If "i" is larger than "0", then set it to the index of the previously passed waypoint in the plan. There is
        no need to check for collisions on the path, in places where the drone has already passed and is not coming
        back to.
        '''
        i = 0
        if self.active_drone_info[drone_id].mission_index > 0:
            i = self.active_drone_info[drone_id].mission_index - 1
        j = i + 1
        collision_detected = False
        p_start = Coordinate()
        p_end = Coordinate()
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
                # Set the return coordinates to be the coord before the first collision, and the first coordinate after
                # the collision. That way the coordinates will span over the whole collision in bound.
                if self.dist_to_dnfz(p3):
                    # This is the first collision detected?:
                    if not collision_detected:
                        collision_detected = True
                        p_start = p1
                    p_end = p2
            i += 1
            j += 1

        if collision_detected:
            return True, p_start, p_end
        else
            return False, None, None

    def pythagoras(self, p1, p2):
        return sqrt((p1.easting - p2.easting)**2 + (p1.northing - p2.northing)**2)

    def is_collision_at_future_position(self, p, t):
        for int_id, dnfz in self.dynamic_no_flight_zones.items():
            if dnfz["geometry"] == "circle":
                coord = dnfz["coordinates"]
                coord = coord.split(',')
                dist = self.pythagoras(p, Coordinate(lon=coord[0], lat=coord[1]))
                if (dnfz["valid_from_epoch"] - self.safety_extra_time) < t < (dnfz["valid_to_epoch"] + self.safety_extra_time):
                    if dist <= (coord[2] + self.safety_dist_to_dnfz):
                        return False, int_id
            elif dnfz["geometry"] == "polygon":
                coords = dnfz["coordinates"]
                coords = coords.split(" ")
                i = 0
                for a in coords:
                    coords[i] = a.split(',')
                    i += 1
                list_of_points = []
                for point in coords:
                    temp_coord = Coordinate(lat=point[1], lon=point[0])
                    list_of_points.append(geometry.Point(temp_coord.easting, temp_coord.northing))
                # Taken from:
                # https://stackoverflow.com/questions/30457089/how-to-create-a-polygon-given-its-point-vertices
                dnfz_polygon = geometry.Polygon([[point.x, point.y] for point in list_of_points])
                dist = dnfz_polygon.distance(geometry.Point(p.easting, p.northing))
                if (dnfz["valid_from_epoch"] - self.safety_extra_time) < t < (dnfz["valid_to_epoch"] + self.safety_extra_time):
                    if dist <= self.safety_dist_to_dnfz:
                        return False, int_id
        return True

if __name__ == "__main__":
    rospy.init_node('collision_detector')  # , anonymous=True)
    rospy.sleep(1)

    cd = CollisionDetector()


#!/usr/bin/env python


from coordinate import Coordinate
import rospy
from gcs.msg import DroneInfo, GPS, DronePath, inCollision
from gcs.srv import safeTakeOff, safeTakeOffResponse
from node_monitor.msg import *
from math import sqrt
import time
import json
import string
from std_msgs.msg import String
from shapely import geometry, wkt

class dictEmpty(dict):
    def __missing__(self, key):
        return 0
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

        self.time_threshold_for_dnfz_in_path_planning = 600  # 10 minutes
        self.time_threshold_for_waiting_on_dnfz = 60    # 60 seconds

        self.FUTURE_COLLISION = inCollision.normal_zone
        self.INSIDE_COLLISION = inCollision.inside_zone
        self.GOAL_COLLISION = inCollision.landing_zone

        # status variables
        rospy.sleep(1)  # wait until everything is running

        self.safe_takeoff_service = rospy.Service("collision_detector/safeTakeOff", safeTakeOff, self.on_safe_takeoff)

        self.collision_detected_pub = rospy.Publisher("/collision_detector/collision_warning", inCollision, queue_size=10)

        # Subscribe to new drone path's, and dynamic obstacles:
        rospy.Subscriber("/gcs/forwardPath", DronePath, self.on_drone_path)
        rospy.Subscriber("/drone_handler/DroneInfo", DroneInfo, self.on_drone_info)
        rospy.Subscriber("/utm/dynamic_no_fly_zones", String, self.on_dynamic_no_fly_zones)
        self.active_drone_paths = dictEmpty()
        self.active_drone_info = dictEmpty()

    def on_drone_path(self, msg):
        '''
        Callback function
        '''
        self.active_drone_paths[msg.DroneID] = msg.Path
        self.calc_dist_between_mission_points(msg.Path, msg.DroneID)

    def on_drone_info(self, msg):
        '''
        Callback function
        '''
        # Adding the next waypoint index in the plan to the dict
        if msg.status == msg.Run:
            self.active_drone_info[msg.drone_id] = msg
        if msg.status == msg.Land:
            # Drone has landed, so delete the path and ID from the dict's
            if (self.active_drone_info[msg.drone_id]) != 0:
                del self.active_drone_info[msg.drone_id]
            if (self.active_drone_paths[msg.drone_id]) != 0:
                del self.active_drone_paths[msg.drone_id]

    def on_dynamic_no_fly_zones(self, msg):
        '''
        Callback function
        Use "int_id" in the string as key in the dict of dnfz
        '''
        try:
            all_json_objs = json.loads(str(msg.data))
            for json_obj in all_json_objs:
                if json_obj["int_id"] in self.dynamic_no_flight_zones:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                else:
                    self.dynamic_no_flight_zones[json_obj["int_id"]] = json_obj
                    if json_obj["geometry"] == "polygon":
                        self.make_polygon(json_obj)
        except ValueError:
            print("Collision Detector: Couldn't convert string from UTM server to json..")

    def on_safe_takeoff(self, req):
        safe_bool, time_left = self.is_clear_to_take_of(req.drone_id)
        return safeTakeOffResponse(safe_bool, time_left)

    def is_clear_to_take_of(self, drone_id):
        start_position = Coordinate(lon=self.active_drone_paths[drone_id][0].longitude,
                                    lat=self.active_drone_paths[drone_id][0].latitude)
        current_time = time.time()
        for int_id, dnfz in self.dynamic_no_flight_zones.items():
            if dnfz["geometry"] == "circle":
                coord = dnfz["coordinates"]
                coord = coord.split(',')
                dist = self.pythagoras(start_position, Coordinate(lon=coord[0], lat=coord[1]))
                if (dnfz["valid_from_epoch"] - self.safety_extra_time) < current_time < (dnfz["valid_to_epoch"] +
                                                                                         self.safety_extra_time):
                    if dist <= (coord[2] + self.safety_dist_to_dnfz):
                        return False, dnfz["valid_to_epoch"]
            elif dnfz["geometry"] == "polygon":
                dist = dnfz["polygon"].distance(geometry.Point(start_position.easting, start_position.northing))
                if (dnfz["valid_from_epoch"] - self.safety_extra_time) < current_time < (dnfz["valid_to_epoch"] +
                                                                                         self.safety_extra_time):
                    if dist <= self.safety_dist_to_dnfz:
                        return False, dnfz["valid_to_epoch"]
        return True, None

    def eta_at_goal(self, drone_id):
        '''
        :returns: amount of seconds until the drone hits the goal
        '''
        speed = self.active_drone_info[drone_id].ground_speed_setpoint
        current_position = Coordinate(GPS_data=self.active_drone_info[drone_id].position)
        if self.active_drone_info[drone_id].mission_index >= self.active_drone_info[drone_id].mission_length:
            # next waypoint is goal.
            goal_position = Coordinate(GPS_data=self.active_drone_paths[drone_id][-1])
            dist = self.pythagoras(current_position, goal_position)
            eta = dist / speed
            return int(eta)
        else:
            next_position = Coordinate(GPS_data=self.active_drone_info[drone_id].next_waypoint)
            dist_to_next = self.pythagoras(current_position, next_position)
            eta = dist_to_next / speed
            for i in range(self.active_drone_info[drone_id].mission_index, self.active_drone_info[drone_id].mission_length-1):
                eta += self.dist_between_mission_points[drone_id][i+1] / speed
            return int(eta)

    def run_collision_check(self):
        '''
        This function will go through all active drones, and look into the future positions to check for possible
        collisions with dynamic obstacle.
        '''
        # For all active drones:
        previous_pos = None
        previous_time = None

        for i, val in self.active_drone_info.items():
            if self.active_drone_paths[i] != 0:
                # Are we in a no-fly-zone now?:
                current_position = Coordinate(GPS_data=val.position)
                collision, int_id = self.is_collision_at_future_position(current_position, time.time())
                if collision:
                    safety_position = self.quick_find_position_outside_dnfz(i, int_id)
                    self.publish_on_ros(i, self.INSIDE_COLLISION, gps_start=safety_position.GPS_data, gps_end=safety_position.GPS_data)
                    break

                # Is there a no-fly-zone on the goal?:
                eta_goal = self.eta_at_goal(i)
                print "Debugging:" , self.active_drone_paths[i]
                goal_position = Coordinate(GPS_data=self.active_drone_paths[i][-1])
                collision, int_id = self.is_collision_at_future_position(goal_position, eta_goal)
                if collision:
                    self.publish_on_ros(i, self.GOAL_COLLISION)

                # for 5, 10, 15, ... , 60 seconds into the future:
                max_future_sight = 60
                if eta_goal < max_future_sight:
                    max_future_sight = eta_goal
                for j in range(5, max_future_sight, 5):
                    position, index = self.calc_future_position(val.drone_id, j)
                    future_time = time.time() + j
                    collision, int_id = self.is_collision_at_future_position(position, future_time)
                    if collision:
                        print("Collision detected in future....")
                        self.make_decision_if_collision(int_id, previous_time, future_time, previous_pos, val.drone_id, index)
                    previous_pos = position
                    previous_time = future_time

    def make_polygon(self, json_obj):
        coords = json_obj["coordinates"]
        coords = coords.split(" ")
        i = 0
        for a in coords:
            coords[i] = a.split(',')
            i += 1
        list_of_points = []
        for point in coords:
            temp_coord = Coordinate(lat=float(point[1]), lon=float(point[0]))
            list_of_points.append(geometry.Point(temp_coord.easting, temp_coord.northing))
        # Taken from:
        # https://stackoverflow.com/questions/30457089/how-to-create-a-polygon-given-its-point-vertices
        dnfz_polygon = geometry.Polygon([[point.x, point.y] for point in list_of_points])
        self.dynamic_no_flight_zones[json_obj["int_id"]]["polygon"] = dnfz_polygon

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
        :returns Future position, mission index for next waypoint
        '''
        current_position = Coordinate(GPS_data=self.active_drone_info[drone_id].position)
        next_position = Coordinate(GPS_data=self.active_drone_info[drone_id].next_waypoint)

        accumulated_mission_dist = self.pythagoras(current_position, next_position)

        dist_to_travel = self.active_drone_info[drone_id].ground_speed_setpoint * sec

        if accumulated_mission_dist > dist_to_travel:
            # future position is before next mission point
            new_easting  = (dist_to_travel * (next_position.easting - current_position.easting) / accumulated_mission_dist) + current_position.easting
            new_northing = (dist_to_travel * (next_position.northing - current_position.northing) / accumulated_mission_dist) + current_position.northing
            return Coordinate(northing=new_northing, easting=new_easting), self.active_drone_info[drone_id].mission_index
        else:
            for i in range(self.active_drone_info[drone_id].mission_index, self.active_drone_info[drone_id].mission_length):
                accumulated_mission_dist += self.dist_between_mission_points[drone_id][i]
                if accumulated_mission_dist > dist_to_travel:
                    # future position is before next mission point
                    resulting_dist = dist_to_travel - accumulated_mission_dist + self.dist_between_mission_points[drone_id][i]
                    p1 = Coordinate(lat=self.active_drone_paths[drone_id][i].latitude, lon=self.active_drone_paths[drone_id][i].longitude)
                    p2 = Coordinate(lat=self.active_drone_paths[drone_id][i+1].latitude, lon=self.active_drone_paths[drone_id][i+1].longitude)
                    new_easting = (resulting_dist * (p2.easting - p1.easting) / self.dist_between_mission_points[drone_id][i]) + p1.easting
                    new_northing = (resulting_dist * (p2.northing - p1.northing) / self.dist_between_mission_points[drone_id][i]) + p1.northing
                    return Coordinate(northing=new_northing, easting=new_easting), i

    def quick_find_position_outside_dnfz(self, drone_id, dnfz_id):
        ''' If a DNFZ pops up on top of the drones position, then the drone should get out as fast as possible. '''
        current_position_coord = Coordinate(GPS_data=self.active_drone_info[drone_id].position)
        new_easting = -1
        new_northing = -1
        print "Json data: ", self.dynamic_no_flight_zones
        if self.dynamic_no_flight_zones[dnfz_id]["geometry"] == "circle":
            coord = self.dynamic_no_flight_zones[dnfz_id]["coordinates"]
            coord = coord.split(',')
            center_coord = Coordinate(lon=coord[0], lat=coord[1])
            ratio_easting, ratio_northing = self.compute_unit_vector(center_coord, current_position_coord)
            new_easting = center_coord.easting + (ratio_easting * (coord[2] + self.safety_dist_to_dnfz))
            new_northing = center_coord.northing + (ratio_northing * (coord[2] + self.safety_dist_to_dnfz))
        elif self.dynamic_no_flight_zones[dnfz_id]["geometry"] == "polygon":
            current_position = geometry.Point(current_position_coord.easting, current_position_coord.northing)
            polygon = self.dynamic_no_flight_zones[dnfz_id]["polygon"]
            exit_point = polygon.exterior.interpolate(polygon.exterior.project(current_position)).wkt
            exit_coord = Coordinate(easting=exit_point.x, northing=exit_point.y)
            ratio_easting, ratio_northing = self.compute_unit_vector(current_position_coord, exit_coord)
            dist = self.pythagoras(current_position_coord, exit_coord) + self.safety_dist_to_dnfz
            new_easting = current_position_coord.easting + (ratio_easting * dist)
            new_northing = current_position_coord.northing + (ratio_northing * dist)
        safety_point = Coordinate(easting=new_easting, northing=new_northing)
        return safety_point

    def compute_unit_vector(self, p_from, p_to):
        dist = self.pythagoras(p_from, p_to)
        d_e = (p_to.easting - p_from.easting) / dist
        d_n = (p_to.northing - p_from.northing) / dist
        return d_e, d_n

    def pythagoras(self, p1, p2):
        return sqrt((p1.easting - p2.easting)**2 + (p1.northing - p2.northing)**2)

    def is_collision_at_future_position(self, p, t):
        for int_id, dnfz in self.dynamic_no_flight_zones.items():
            if dnfz["geometry"] == "circle":
                coord = dnfz["coordinates"]
                coord = coord.split(',')
                dist = self.pythagoras(p, Coordinate(lon=coord[0], lat=coord[1]))
                if (int(dnfz["valid_from_epoch"]) - self.safety_extra_time) < t < (int(dnfz["valid_to_epoch"]) + self.safety_extra_time):
                    if dist <= (int(coord[2]) + self.safety_dist_to_dnfz):
                        return False, int_id
            elif dnfz["geometry"] == "polygon":
                dist = dnfz["polygon"].distance(geometry.Point(p.easting, p.northing))
                if (int(dnfz["valid_from_epoch"]) - self.safety_extra_time) < t < (int(dnfz["valid_to_epoch"]) + self.safety_extra_time):
                    if dist <= self.safety_dist_to_dnfz:
                        return False, int_id
        return True, None

    def make_decision_if_collision(self, dnfz_id, clear_time, coll_time, pos_clear, drone_id, previous_mission_index):
        pos_pre_collision = pos_clear
        pos_post_collision = pos_clear
        next_mission_index = -1
        if (self.dynamic_no_flight_zones[dnfz_id]["valid_to_epoch"] - coll_time) > self.time_threshold_for_waiting_on_dnfz:
            print("New path plan is required...")
            collision_time = coll_time - time.time()
            i = 1
            while True:
                collision_time += i
                pos_post_collision, next_mission_index = self.calc_future_position(drone_id, i)
                collision, int_id = self.is_collision_at_future_position(pos_post_collision, collision_time)
                if not collision:
                    break
                i += 1
            clear_time = 0
        else:
            clear_time = self.dynamic_no_flight_zones[dnfz_id]["valid_to_epoch"] + self.time_threshold_for_waiting_on_dnfz
        start_GPS = pos_pre_collision.GPS_data
        end_GPS = pos_post_collision.GPS_data

        self.publish_on_ros(drone_id, previous_mission_index, next_mission_index, start_GPS, end_GPS, clear_time)

    def publish_on_ros(self, drone_id, zone_type, index1=-1, index2=-1, gps_start=None, gps_end=None, valid_to=0):
        if gps_start is None or gps_end is None:
            gps_start = GPS()
            gps_end = GPS()
        msg = inCollision()
        msg.drone_id = drone_id
        msg.zone_type = zone_type
        msg.plan_index1 = index1
        msg.plan_index2 = index2
        msg.start = gps_start
        msg.end = gps_end
        msg.valid_to = valid_to
        self.collision_detected_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('collision_detector')  # , anonymous=True)
    rospy.sleep(1)

    cd = CollisionDetector()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size=10)

    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'collision_detector'
    heart_msg.rate = 1

    while not rospy.is_shutdown():
        cd.run_collision_check()
        rospy.Rate(heart_msg.rate).sleep()
        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)

# TODO: Remarks: Not sure any of the Shapely thing works.
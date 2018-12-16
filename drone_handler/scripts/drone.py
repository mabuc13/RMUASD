from collections import deque
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from telemetry.srv import * # pylint: disable=W0614
from std_srvs.srv import Trigger
from std_msgs.msg import Int16, Empty
from gcs.msg import GPS, DroneInfo
import flight_modes
import rospy
import manual_mission
from enum import Enum
 
# defines 
BUFFER_SIZE = 30
DESIRED_RELATIVE_ALT = 20
MISSION_SPEED = 5  # m/s
WP_ACCEPTANCE_RADIUS = 10
MAX_UPLOAD_RETRIES = 3
UPLOAD_DELAY = 5
HOLD_TIME = 15
GOAL_DISTANCE = 10

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_LAND = 21
MAV_CMD_DO_CHANGE_SPEED = 178

MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

class State(Enum):
    GROUNDED = 1
    REQUESTING_UPLOAD = 2
    UPLOADING = 3
    TAKEOFF = 4
    SYNC_WP_IDX = 5
    ARMING = 6
    SET_MISSION = 7
    FLYING_MISSION = 8
    PAUSED = 9
    LANDING = 10
    SET_SPEED = 11
    REPOSITION = 12
    CLEARING_MISSION = 13
    WAITING = 14
    HOLD = 15

PAUSE_LIST_MAIN = ["Manual", "Stabilized", "Altitude Control", "Position Control", "Rattitude", "Acro"]
PAUSE_LIST_SUB  = ["Return to Home", "Follow Me"]

class Drone(object):

    def __init__(self, drone_id=1):
        # All variables relating to the status and information of the drone

        self.id = drone_id
        self.up_time = 0
        self.gps_timestamp = 0

        self.fsm_state = State.GROUNDED
        self.new_mission = False
        self.upload_done = False
        self.upload_failed = False
        self.cmd_try_again = False
        self.speed_ack = False
        self.loiter = False
        self.active = False
        self.wait = False
        self.hold = False
        self.upload_retries = 0

        self.mission_id = 0
        self.pending_mission_gps = []
        self.active_waypoint_gps = GPS()
        self.active_mission_gps = []
        self.active_mission_ml  = mavlink_lora_mission_list()
        self.active_mission_len = 0
        self.active_mission_idx = 0 
        self.dist_to_goal = -1

        self.holding_waypoint = GPS()

        self.gcs_status = DroneInfo.Land

        # from heartbeat status
        self.main_mode = ""
        self.sub_mode = ""
        self.autopilot = ""
        self.type = ""
        self.state = ""

        # from mav mode
        self.armed = False
        self.manual_input = False
        self.hil_simulation = False
        self.stabilized_mode = False
        self.guided_mode = False
        self.auto_mode = False
        self.test_mode = False
        self.custom_mode = False

        # from statustext
        self.statustext = deque([], maxlen=BUFFER_SIZE)
        self.severity = deque([], maxlen=BUFFER_SIZE)

        # from vfr hud
        self.ground_speed = 0
        self.climb_rate = 0
        self.throttle = 0

        # from home position
        self.home_position = GPS()

        # from drone attitude
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # from drone status
        self.battery_voltage = 0
        self.battery_SOC = 0
        self.cpu_load = 0
        self.msg_sent_gcs = 0
        self.msg_received_gcs = 0
        self.msg_dropped_gcs = 0
        self.msg_dropped_uas = 0
        self.last_heard = rospy.Time().now()

        # from drone pos
        self.latitude = 0
        self.longitude = 0
        self.absolute_alt = 0
        self.relative_alt = 0
        self.heading = 0

        self.clear_mission_pub = rospy.Publisher("/mavlink_interface/mission/mavlink_clear_all", Empty, queue_size=0)
        self.upload_mission_pub = rospy.Publisher("/telemetry/new_mission", mavlink_lora_mission_list, queue_size=0)
        self.set_current_mission_pub = rospy.Publisher("/telemetry/mission_set_current", Int16, queue_size=10)

        # TODO add drone id to services
        self.arm                = rospy.ServiceProxy("/telemetry/arm_drone", Trigger)
        self.disarm             = rospy.ServiceProxy("/telemetry/disarm_drone", Trigger)
        self.takeoff            = rospy.ServiceProxy("/telemetry/takeoff_drone", TakeoffDrone)
        self.land               = rospy.ServiceProxy("/telemetry/land_drone", LandDrone)
        self.set_mode           = rospy.ServiceProxy("/telemetry/set_mode", SetMode)
        self.set_home           = rospy.ServiceProxy("/telemetry/set_home", SetHome)
        self.change_speed       = rospy.ServiceProxy("/telemetry/change_speed", ChangeSpeed)
        self.return_to_home     = rospy.ServiceProxy("/telemetry/return_home", Trigger)
        self.reposition         = rospy.ServiceProxy("/telemetry/goto_waypoint", GotoWaypoint)
        self.upload_mission     = rospy.ServiceProxy("/telemetry/upload_mission", UploadMission)

        # class that handles the missions manually while an upload is in progress
        self.manual_mission = manual_mission.ManualMission(target_sys=self.id, target_comp=0, reposition_handle=self.reposition)

    def on_move(self, msg):
        # reset manual mission
        # reposition
        # change state to reposition

        self.manual_mission.stop_running()
        self.manual_mission.reset()

        request = GotoWaypointRequest(
            relative_alt=True,
            ground_speed=MISSION_SPEED,
            latitude=msg.position.latitude,
            longitude=msg.position.longitude,
            altitude=msg.position.altitude
        )

        self.reposition(request)
        
        self.holding_waypoint = msg.position
        self.fsm_state = State.REPOSITION

    def on_mission_ack(self, msg):
        if msg.result_text == "MAV_MISSION_ACCEPTED":
            self.upload_done = True
            self.upload_retries = 0
        else:
            # restart upload
            if self.upload_retries < MAX_UPLOAD_RETRIES:
                self.upload_failed = True
                self.upload_retries += 1
            else:
                # TODO Handle the failure scenario
                rospy.logwarn("Mission upload failed - giving up.")

    def on_command_ack(self, msg):
        if msg.command == MAV_CMD_DO_CHANGE_SPEED and msg.result == 0:
            self.speed_ack = True

    def on_cmd_fail(self, msg):
        self.cmd_try_again = True
        self.manual_mission.on_cmd_fail()

    def calc_remaining_distance(self):
        if self.active_mission_len > 0:
            lat_goal = self.active_mission_gps[-1].latitude
            lon_goal = self.active_mission_gps[-1].longitude

            self.dist_to_goal = manual_mission.distGreatCircle(self.latitude, self.longitude, lat_goal, lon_goal)
            # print("Distance to goal: {}".format(self.dist_to_goal))
        else:
            self.dist_to_goal = -1

    def update_mission(self, path):
        self.pending_mission_gps = path

    def start_mission(self, loiter=False):        
        self.loiter = loiter
        self.active_mission_gps = self.pending_mission_gps
        self.active_mission_ml = self.gps_to_mavlink(self.pending_mission_gps)
        self.active_mission_len = len(self.active_mission_ml.waypoints)

        self.manual_mission.stop_running()
        self.manual_mission.reset()
        self.manual_mission.update_mission(self.active_mission_gps)
        self.new_mission = True
        
        print("New mission")

    def gps_to_mavlink(self, gps_list):
        sequence_number = 1

        ml_list = mavlink_lora_mission_list()

        for itr, waypoint in enumerate(gps_list):
            current = 0
            if itr == 0:
                current = 1

            mission_item = mavlink_lora_mission_item_int(
                param1=0,                       # hold time in seconds
                param2=WP_ACCEPTANCE_RADIUS,    # acceptance radius [m]
                param3=0,                       # orbit CW or CCW
                param4=float('nan'),
                x=int(waypoint.latitude*1e7),
                y=int(waypoint.longitude*1e7),
                z=waypoint.altitude,
                seq=sequence_number,
                command=MAV_CMD_NAV_WAYPOINT,
                current=current,
                autocontinue=1,
                target_system=self.id,
                target_component=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            )
            sequence_number += 1
            ml_list.waypoints.append(mission_item)

        if self.loiter:
            # set last waypoint to a loiter command
            ml_list.waypoints[-1].command = MAV_CMD_NAV_LOITER_UNLIM
            ml_list.waypoints[-1].param2 = 0 
            ml_list.waypoints[-1].autocontinue = 0
        else:
            # set last waypoint to a landing command
            ml_list.waypoints[-1].command = MAV_CMD_NAV_LAND
            ml_list.waypoints[-1].param1 = 0        # abort alt
            ml_list.waypoints[-1].param2 = 2        # precision land
            ml_list.waypoints[-1].z = 0
            ml_list.waypoints[-1].autocontinue = 0
        ml_list.header.stamp = rospy.Time.now()

        return ml_list

    def is_on_goal(self):
        if self.active_mission_idx == self.active_mission_len - 1: 
            if self.active_mission_ml.waypoints[-1].command == MAV_CMD_NAV_LAND:
                if self.dist_to_goal < GOAL_DISTANCE:
                    return True

        return False

    def wait_cb(self, event):
        self.wait = False

    def hold_cb(self, event):
        self.hold = False

    def reset(self):
        self.upload_done = False
        self.upload_failed = False
        self.cmd_try_again = False
        self.new_mission = False
        self.speed_ack = False

    def run(self):

        if self.active_mission_len > 0:
            try:
                if self.manual_mission.fsm_state == manual_mission.State.IDLE:
                    self.active_waypoint_gps = self.active_mission_gps[self.active_mission_idx]
                else:
                    self.active_waypoint_gps = self.manual_mission.mission[self.manual_mission.mission_idx]
            except IndexError as err:
                rospy.logwarn(err)
                rospy.logwarn("Can't assign active waypoint. Mission is not up to date yet.")

        if self.main_mode in PAUSE_LIST_MAIN or self.sub_mode in PAUSE_LIST_SUB:
            # print("Setting paused!")
            self.fsm_state = State.PAUSED

        # ------------------------------------------------------------------------------ #
        if self.fsm_state == State.GROUNDED:
            if self.new_mission:
                # prepare for upload and change state
                self.active_mission_gps = self.pending_mission_gps
                self.active_mission_ml = self.gps_to_mavlink(self.active_mission_gps)

                self.new_mission = False
                self.fsm_state = State.CLEARING_MISSION
                self.clear_mission_pub.publish(Empty())
            
        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.REQUESTING_UPLOAD:
            request = UploadMissionRequest(drone_id=self.id, waypoints=self.active_mission_ml.waypoints)
            response = self.upload_mission(request)
            
            if response.success:
                self.fsm_state = State.UPLOADING
                self.upload_done = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.UPLOADING:
            # only a positive ack counts as a succesful upload (see callback)
            if self.upload_done:
                if self.state == "Standby":
                    request = TakeoffDroneRequest(on_the_spot=True, alt=20)
                    response = self.takeoff(request)
                    if response.success:
                        self.fsm_state = State.TAKEOFF

                elif self.state == "Active":
                    self.manual_mission.stop_running()
                    self.manual_mission.reset()
                    self.fsm_state = State.SYNC_WP_IDX

            elif self.upload_failed:
                rospy.Timer(rospy.Duration(UPLOAD_DELAY), self.wait_cb, oneshot=True)
                self.wait = True
                self.fsm_state = State.WAITING
                self.upload_failed = False


        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.TAKEOFF:
            if self.sub_mode == "Takeoff":
                response = self.arm()
                if response.success:
                    self.fsm_state = State.ARMING

            else:
                if self.cmd_try_again:
                    request = TakeoffDroneRequest(on_the_spot=True, alt=20)
                    response = self.takeoff(request)
                    if response.success:
                        self.cmd_try_again = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.ARMING:
            if self.armed and self.relative_alt > 19:
                self.gcs_status = DroneInfo.Run
                response = self.set_mode(flight_modes.MISSION)
                if response.success:
                    self.fsm_state = State.SET_MISSION

            else:
                if self.cmd_try_again:
                    response = self.arm()
                    if response.success:
                        self.cmd_try_again = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.SYNC_WP_IDX:

            if self.manual_mission.mission_idx == self.active_mission_idx:
                response = self.set_mode(flight_modes.MISSION)
                if response.success:
                    self.fsm_state = State.SET_MISSION
            else:
                print("[drone]: Manual idx = {}, Active idx = {}".format(self.manual_mission.mission_idx, self.active_mission_idx))
                self.set_current_mission_pub.publish(Int16(self.manual_mission.mission_idx))

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.SET_MISSION:
            if self.sub_mode == "Mission":
                self.gcs_status = DroneInfo.Run
                request = ChangeSpeedRequest(MISSION_SPEED)
                response = self.change_speed(request)
                if response.success:
                    self.fsm_state = State.SET_SPEED
            else:
                if self.cmd_try_again:
                    response = self.set_mode(flight_modes.MISSION)
                    if response.success:
                        self.cmd_try_again = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.SET_SPEED:
            if self.speed_ack:
                self.fsm_state = State.FLYING_MISSION
                self.speed_ack = False
            else:
                if self.cmd_try_again:
                    request = ChangeSpeedRequest(MISSION_SPEED)
                    response = self.change_speed(request)
                    if response.success:
                        self.cmd_try_again = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.FLYING_MISSION:
            
            if self.new_mission:
                self.manual_mission.start_running()
                self.active_mission_idx = 0
                self.new_mission = False
            # elif self.manual_mission.fsm_state == manual_mission.State.ON_THE_WAY:
                self.fsm_state = State.CLEARING_MISSION
                self.clear_mission_pub.publish(Empty())

            if self.is_on_goal():
                # self.fsm_state = State.LANDING
                lat_goal = self.active_mission_gps[-1].latitude
                lon_goal = self.active_mission_gps[-1].longitude
                # reposition to a lower altitude above the goal
                request = GotoWaypointRequest(
                    relative_alt=True,
                    ground_speed=MISSION_SPEED,
                    latitude=lat_goal,
                    longitude=lon_goal,
                    altitude=10
                )
                                
                response = self.reposition(request)
                if response.success:
                    rospy.Timer(rospy.Duration(HOLD_TIME), self.hold_cb, oneshot=True)
                    self.hold = True
                    self.fsm_state = State.HOLD

            elif self.loiter and self.ground_speed < 0.5:
                self.gcs_status = DroneInfo.holding

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.REPOSITION:

            if self.new_mission:
                self.manual_mission.start_running()
                self.active_mission_idx = 0
                self.new_mission = False
                # elif self.manual_mission.fsm_state == manual_mission.State.ON_THE_WAY:
                self.fsm_state = State.CLEARING_MISSION
                self.clear_mission_pub.publish(Empty())

            else:
                if self.cmd_try_again:
                    request = GotoWaypointRequest(
                        relative_alt=True,
                        ground_speed=MISSION_SPEED,
                        latitude=self.holding_waypoint.position.latitude,
                        longitude=self.holding_waypoint.position.longitude,
                        altitude=self.holding_waypoint.position.altitude
                    )

                    response = self.reposition(request)
                    if response.success:
                        self.cmd_try_again = False
        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.LANDING:
            if self.state == "Standby":
                self.fsm_state = State.GROUNDED
                self.gcs_status = DroneInfo.Land
                # reset states
                self.reset()
                self.manual_mission.reset()
            else:
                if self.cmd_try_again:
                    response = self.set_mode(flight_modes.MISSION)
                    if response.success:
                        self.cmd_try_again = False
            
        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.CLEARING_MISSION:
            if self.upload_done:
                self.upload_done = False
                self.fsm_state = State.REQUESTING_UPLOAD

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.PAUSED:
            if self.state == "Active" and self.sub_mode == "Mission":
                self.fsm_state = State.FLYING_MISSION
            elif self.state == "Standby" or not self.armed:
                self.fsm_state = State.GROUNDED

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.WAITING:
            if not self.wait:
                self.fsm_state = State.REQUESTING_UPLOAD

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.HOLD:
            if not self.hold:
                response = self.set_mode(flight_modes.MISSION)
                if response.success:
                    self.fsm_state = State.LANDING

            else:
                if self.cmd_try_again:
                    lat_goal = self.active_mission_gps[-1].latitude
                    lon_goal = self.active_mission_gps[-1].longitude
                    # reposition to a lower altitude above the goal
                    request = GotoWaypointRequest(
                        relative_alt=True,
                        ground_speed=MISSION_SPEED,
                        latitude=lat_goal,
                        longitude=lon_goal,
                        altitude=10
                    )
                                    
                    response = self.reposition(request)
                    if response.success:
                        self.cmd_try_again = False



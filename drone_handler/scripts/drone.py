from collections import deque
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from telemetry.srv import * # pylint: disable=W0614
from std_srvs.srv import Trigger

from gcs.msg import GPS, DroneInfo
import flight_modes
import rospy
from enum import Enum
 
# defines 
BUFFER_SIZE = 30
DESIRED_RELATIVE_ALT = 20

MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_LAND = 21
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

class State(Enum):
    GROUNDED = 1
    REQUESTING_UPLOAD = 2
    UPLOADING = 3
    TAKEOFF = 4
    REPOSITION = 5
    ARMING = 6
    SET_MISSION = 7
    FLYING_MISSION = 8
    PAUSED = 9
    LANDING = 10

PAUSE_LIST_MAIN = ["Manual", "Stabilized", "Altitude Control", "Position Control", "Rattitude", "Acro"]
PAUSE_LIST_SUB  = ["Return to Home", "Follow Me"]

class Drone(object):

    def __init__(self, drone_id=1):
        # All variables relating to the status and information of the drone

        self.id = drone_id
        self.up_time = 0

        self.fsm_state = State.GROUNDED
        self.new_mission = False
        self.upload_done = False
        self.cmd_try_again = False

        self.pending_mission_gps = []
        self.active_waypoint_gps = GPS()
        self.active_mission_gps = []
        self.active_mission_len = 0
        self.active_mission_ml  = mavlink_lora_mission_list()
        self.active_sub_mission = mavlink_lora_mission_list()
        self.active_mission_idx = 0             # index for the complete plan
        self.active_sub_mission_offset = 0      # at what index does the sub plan start
        self.swapping_mission = False
        self.running_sub_missions = False
        
        # the drone starts on the ground ( land means ground for now. Change later ) TODO
        self.gcs_status = DroneInfo.Land

        # from mission info
        self.active_sub_waypoint_idx = 0
        self.active_sub_mission_len = 0
        self.active_sub_waypoint = mavlink_lora_mission_item_int()

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
        self.battery_volt = 0
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

        # Maybe have service proxies in here so that drone operations seem like functions called on the drone object

        self.upload_mission_pub = rospy.Publisher("/telemetry/new_mission", mavlink_lora_mission_list, queue_size=0)

        # TODO add drone id to services
        self.arm                = rospy.ServiceProxy("/telemetry/arm_drone", Trigger)
        self.disarm             = rospy.ServiceProxy("/telemetry/disarm_drone", Trigger)
        self.takeoff            = rospy.ServiceProxy("/telemetry/takeoff_drone", TakeoffDrone)
        self.land               = rospy.ServiceProxy("/telemetry/land_drone", LandDrone)
        self.set_mode           = rospy.ServiceProxy("/telemetry/set_mode", SetMode)
        self.set_home           = rospy.ServiceProxy("/telemetry/set_home", SetHome)
        self.return_to_home     = rospy.ServiceProxy("/telemetry/return_home", Trigger)
        self.reposition         = rospy.ServiceProxy("/telemetry/goto_waypoint", GotoWaypoint)
        self.upload_mission     = rospy.ServiceProxy("/telemetry/upload_mission", UploadMission)

    def on_mission_ack(self, msg):
        if msg.result_text == "MAV_MISSION_ACCEPTED":
            self.upload_done = True
        else:
            # restart upload
            self.fsm_state = State.REQUESTING_UPLOAD

    def on_cmd_fail(self, msg):
        self.cmd_try_again = True

    def update_mission(self, path):
        self.pending_mission_gps = path
        # self.active_mission_len = len(self.pending_mission_gps)
        # self.new_mission = True

    def start_mission(self):
        self.new_mission = True

        if self.active_mission_len > 0:
            self.running_sub_missions = True
        self.active_mission_len = len(self.pending_mission_gps)
        self.active_sub_mission_offset = 0
        print("New mission")

    def gps_to_mavlink(self, gps_list):
        sequence_number = 0

        ml_list = mavlink_lora_mission_list()
        for itr, waypoint in enumerate(gps_list):
            current = 0
            if itr == 0:
                current = 1

            mission_item = mavlink_lora_mission_item_int(
                param1=0,                       # hold time in seconds
                param2=2,                       # acceptance radius [m]
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

        # set last waypoint to a landing command
        ml_list.waypoints[-1].command = MAV_CMD_NAV_LAND
        ml_list.waypoints[-1].param1 = 0        # abort alt
        ml_list.waypoints[-1].param2 = 2        # precision land
        ml_list.waypoints[-1].z = 0
        ml_list.waypoints[-1].autocontinue = 1
        ml_list.header.stamp = rospy.Time.now()

        return ml_list

    def next_sub_mission(self, new_mission=False):
        offset = 0
        if not new_mission:
            offset = self.active_sub_mission_offset + 2
        
        if offset > self.active_mission_len - 3:
            self.active_sub_mission.waypoints = self.active_mission_ml.waypoints[offset : self.active_mission_len]
        else:
            self.active_sub_mission.waypoints = self.active_mission_ml.waypoints[offset : offset+3]

        # make sure the first waypoint is active!
        self.active_sub_mission.waypoints[0].current = 1
        self.active_sub_mission_offset = offset
        # self.active_sub_waypoint_idx = 0

    def run(self):

        if self.main_mode in PAUSE_LIST_MAIN or self.sub_mode in PAUSE_LIST_SUB:
            self.fsm_state = State.PAUSED

        # the sub waypoint idx can only be trusted when we are not swapping missions
        if not self.swapping_mission:
            self.active_mission_idx = self.active_sub_mission_offset + self.active_sub_waypoint_idx
            try:
                self.active_sub_waypoint = self.active_sub_mission.waypoints[self.active_sub_waypoint_idx]
                self.active_waypoint_gps = self.active_mission_gps[self.active_mission_idx]
            except:
                pass

        # ------------------------------------------------------------------------------ #
        if self.fsm_state == State.GROUNDED:
            if self.new_mission:
                # prepare for upload and change state
                self.active_mission_gps = self.pending_mission_gps
                self.active_mission_ml = self.gps_to_mavlink(self.active_mission_gps)
                # from the grounded state, the sub mission is the same as the whole mission
                self.active_sub_mission = self.active_mission_ml

                self.new_mission = False
                self.fsm_state = State.REQUESTING_UPLOAD
            
        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.REQUESTING_UPLOAD:
            request = UploadMissionRequest(drone_id=self.id, waypoints=self.active_sub_mission.waypoints)
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
                    response = self.set_mode(flight_modes.MISSION)
                    if response.success:
                        self.fsm_state = State.SET_MISSION

                else:
                    self.fsm_state = State.PAUSED
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
            if self.armed:
                if self.sub_mode == "Loiter" or self.relative_alt > 19:
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
        elif self.fsm_state == State.SET_MISSION:
            if self.sub_mode == "Mission":
                self.fsm_state = State.FLYING_MISSION
            else:
                if self.cmd_try_again:
                    response = self.set_mode(flight_modes.MISSION)
                    if response.success:
                        self.cmd_try_again = False
        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.FLYING_MISSION:

            if self.new_mission:
                # reposition to first wp of new mission
                self.active_mission_gps = self.pending_mission_gps
                self.active_mission_ml = self.gps_to_mavlink(self.active_mission_gps)
                # only use the first 3 waypoints in the sub mission
                self.active_sub_mission_offset = 0
                self.next_sub_mission(new_mission=True)
                # self.active_sub_mission.waypoints = self.active_mission_ml.waypoints[0:3]
                first_wp = self.active_mission_gps[0]
                request = GotoWaypointRequest(
                    relative_alt=True, latitude=first_wp.latitude,
                    longitude=first_wp.longitude, altitude=first_wp.altitude
                    )
                response = self.reposition(request)

                if response.success:
                    self.fsm_state = State.REPOSITION
                    self.new_mission = False

            if self.active_sub_waypoint_idx == 0:
                self.swapping_mission = False
                
            
            # check mission progress to see if new sub mission needs to be uploaded
            
            # only swap if there is a valid mission, and we are not already swapping
            if self.active_sub_mission_len > 0 and not self.swapping_mission:
                # only swap if there is room in the mission plan
                if self.active_sub_mission_offset < self.active_mission_len - 3:
                    # only swap at the end of a mission if we know it's a sub mission
                    if self.running_sub_missions:
                        # start swapping when the last waypoint in the sub plan becomes active
                        if self.active_sub_waypoint_idx > self.active_sub_mission_len - 2:
                            # print(self.active_sub_waypoint_idx)
                            self.swapping_mission = True
                            self.next_sub_mission()
                            self.fsm_state = State.REQUESTING_UPLOAD


            if self.active_mission_idx == self.active_mission_len - 1 and self.relative_alt < 15:
                # request = LandDroneRequest(on_the_spot=False, precision_land=1, yaw=-1,
                #     lat=self.active_waypoint_gps.latitude, lon=self.active_waypoint_gps.longitude ,alt=0)
                # response = self.land(request)
                # if response.success:
                #     self.fsm_state = State.LANDING
                self.fsm_state = State.LANDING

        elif self.fsm_state == State.LANDING:
            if self.state == "Standby":
                self.fsm_state = State.GROUNDED
                self.gcs_status = DroneInfo.Land
                # self.active_mission_ml  = mavlink_lora_mission_list()
                # self.active_sub_mission = mavlink_lora_mission_list()
                # self.active_mission_len = 0
                # self.active_mission_idx = 0
                # self.active_sub_mission_offset = 0  
            # else:
            #     if self.cmd_try_again:
            #         request = LandDroneRequest(on_the_spot=False, precision_land=1, yaw=-1,
            #             lat=self.active_waypoint_gps.latitude, lon=self.active_waypoint_gps.longitude ,alt=0)
            #         response = self.land(request)
            #         if response.success:
            #             self.cmd_try_again = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.REPOSITION:
            if self.sub_mode == "Loiter":
                self.fsm_state = State.REQUESTING_UPLOAD
            else:
                if self.cmd_try_again:
                    first_wp = self.active_mission_gps[0]
                    request = GotoWaypointRequest(
                        relative_alt=True, latitude=first_wp.latitude,
                        longitude=first_wp.longitude, altitude=first_wp.altitude
                        )
                    response = self.reposition(request)
                    if response.success:
                        self.cmd_try_again = False
            
        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.PAUSED:
            if self.sub_mode == "Mission":
                self.fsm_state = State.FLYING_MISSION
            elif self.state == "Standby" or not self.armed:
                self.fsm_state = State.GROUNDED
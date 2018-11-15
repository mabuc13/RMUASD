from collections import deque
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from telemetry.srv import * # pylint: disable=W0614
from std_srvs.srv import Trigger
from std_msgs.msg import Int16
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
        self.active_mission_ml  = mavlink_lora_mission_list()
        self.active_sub_mission = mavlink_lora_mission_list()
        self.active_mission_len = 0
        self.active_mission_idx = 0             # index for the complete plan


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

        self.upload_mission_pub = rospy.Publisher("/telemetry/new_mission", mavlink_lora_mission_list, queue_size=0)
        self.set_current_mission_pub = rospy.Publisher("/telemetry/mission_set_current", Int16, queue_size=10)

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

        # class that handles the missions manually while an upload is in progress
        self.manual_mission = manual_mission.ManualMission(target_sys=self.id, target_comp=0, reposition_handle=self.reposition)

    def on_mission_ack(self, msg):
        if msg.result_text == "MAV_MISSION_ACCEPTED":
            self.upload_done = True
        else:
            # restart upload
            self.fsm_state = State.REQUESTING_UPLOAD

    def on_cmd_fail(self, msg):
        self.cmd_try_again = True
        self.manual_mission.on_cmd_fail()

    def update_mission(self, path):
        self.pending_mission_gps = path

    def start_mission(self):

        # if the drone is already flying, it has to start running sub missions
        if self.state == "Active":
            self.running_sub_missions = True
        
        self.active_mission_gps = self.pending_mission_gps
        self.active_mission_ml = self.gps_to_mavlink(self.pending_mission_gps)
        self.active_mission_len = len(self.active_mission_ml.waypoints)
        self.active_sub_mission_offset = 0

        self.manual_mission.update_mission(self.active_mission_gps)
        self.new_mission = True
        
        print("New mission")

    def gps_to_mavlink(self, gps_list):
        sequence_number = 1

        ml_list = mavlink_lora_mission_list()

        # insert speed command as first item
        speed_cmd = mavlink_lora_mission_item_int(
                param1=1,                       # speed type - ground speed
                param2=MISSION_SPEED,           # speed - m/s
                param3=-1,                      # throttle
                param4=1,                       # absolute speed
                x=0,
                y=0,
                z=0,
                seq=0,
                command=MAV_CMD_DO_CHANGE_SPEED,
                current=1,
                autocontinue=1,
                target_system=self.id,
                target_component=0,
                frame=MAV_FRAME_MISSION
        )

        ml_list.waypoints.append(speed_cmd)

        for itr, waypoint in enumerate(gps_list):
            current = 0
            # if itr == 0:
            #     current = 1

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

        # set last waypoint to a landing command
        ml_list.waypoints[-1].command = MAV_CMD_NAV_LAND
        ml_list.waypoints[-1].param1 = 0        # abort alt
        ml_list.waypoints[-1].param2 = 2        # precision land
        ml_list.waypoints[-1].z = 0
        ml_list.waypoints[-1].autocontinue = 0
        ml_list.header.stamp = rospy.Time.now()

        return ml_list

    def run(self):

        if self.main_mode in PAUSE_LIST_MAIN or self.sub_mode in PAUSE_LIST_SUB:
            self.fsm_state = State.PAUSED

        # ------------------------------------------------------------------------------ #
        if self.fsm_state == State.GROUNDED:
            if self.new_mission:
                # prepare for upload and change state
                self.active_mission_gps = self.pending_mission_gps
                self.active_mission_ml = self.gps_to_mavlink(self.active_mission_gps)
                # from the grounded state, the sub mission is the same as the whole mission
                self.active_sub_mission = self.active_mission_ml

                self.running_sub_missions = False
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
                        self.manual_mission.stop_running()
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
                self.manual_mission.start_running()
                self.fsm_state = State.REQUESTING_UPLOAD
                self.new_mission = False

            if self.active_mission_idx == self.active_mission_len - 1 and self.relative_alt < 20:
                self.fsm_state = State.LANDING

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.LANDING:
            if self.state == "Standby":
                self.fsm_state = State.GROUNDED
                self.gcs_status = DroneInfo.Land

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
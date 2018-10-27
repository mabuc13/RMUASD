from collections import deque
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from telemetry.srv import * # pylint: disable=W0614
from std_srvs.srv import Trigger

from gcs.msg import GPS
import flight_modes
import rospy
from enum import Enum
 
# defines 
BUFFER_SIZE = 30
DESIRED_RELATIVE_ALT = 20

MAV_CMD_NAV_WAYPOINT = 16
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

class State(Enum):
    GROUNDED = 1
    STANDBY_UPLOAD = 2
    TAKEOFF = 3
    REPOSITION = 4
    ACTIVE_UPLOAD = 5
    SET_MISSION = 6
    FLYING_MISSION = 7
    PAUSED = 8
    REQUESTING_UPLOAD = 9
    ARMING = 10

class Drone(object):

    def __init__(self, drone_id=1):
        # All variables relating to the status and information of the drone

        self.id = drone_id
        self.last_heard = 0

        self.fsm_state = State.GROUNDED
        self.new_mission = False
        self.upload_done = False
        self.cmd_try_again = False

        self.pending_mission_gps = []
        self.pending_sub_mission = []
        self.active_mission_gps = []
        self.active_mission_ml  = mavlink_lora_mission_list()
        self.active_sub_mission = mavlink_lora_mission_list()
        self.active_mission_idx = 0
        self.active_sub_mission_offset = 0
        self.mission_locked = False
        self.desired_relative_alt = DESIRED_RELATIVE_ALT

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

        # from mission info
        self.active_sub_waypoint_idx = 0
        self.active_sub_mission_len = 0
        self.active_sub_waypoint = mavlink_lora_mission_item_int()

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
        # self.new_mission = True

    def start_mission(self):
        self.new_mission = True
        print("New mission")

    def gps_to_mavlink(self):
        sequence_number = 0
        self.active_mission_ml.waypoints.clear()
        for waypoint in self.pending_mission_gps:
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
                current=0,
                autocontinue=1,
                target_system=self.id,
                target_component=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            )
            sequence_number += 1
            self.active_mission_ml.waypoints.append(mission_item)
        self.active_mission_ml.header.stamp = rospy.Time.now()

    def run(self):

        if self.main_mode == "Stabilized" or self.main_mode == "Manual":
            self.fsm_state = State.PAUSED

        if self.fsm_state == State.GROUNDED:
            if self.new_mission:
                # begin upload and change state
                self.active_mission_gps = self.pending_mission_gps
                self.gps_to_mavlink()
                
                # request = UploadMissionRequest(drone_id=self.id, waypoints=self.active_mission_ml.waypoints)
                # response = self.upload_mission(request)
                self.new_mission = False
                self.fsm_state = State.REQUESTING_UPLOAD
                
                # if response.success:
                #     self.fsm_state = State.STANDBY_UPLOAD
                # else:
                #     self.fsm_state = State.REQUESTING_UPLOAD
            
            pass
        
        elif self.fsm_state == State.REQUESTING_UPLOAD:
            request = UploadMissionRequest(drone_id=self.id, waypoints=self.active_mission_ml.waypoints)
            response = self.upload_mission(request)
            
            if response.success:
                if self.state == "Standby":
                    self.fsm_state = State.STANDBY_UPLOAD
                elif self.state == "Active":
                    self.fsm_state = State.ACTIVE_UPLOAD
                else:
                    self.fsm_state == State.PAUSED

                self.upload_done = False

        elif self.fsm_state == State.STANDBY_UPLOAD:
            if self.upload_done:
                request = TakeoffDroneRequest(on_the_spot=True, alt=20)
                response = self.takeoff(request)

                if response.success:
                    self.fsm_state = State.TAKEOFF

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

        elif self.fsm_state == State.ARMING:
            if self.armed:
                if self.sub_mode == "Loiter":
                    response = self.set_mode(flight_modes.MISSION)

                    if response.success:
                        self.fsm_state = State.SET_MISSION

            else:
                if self.cmd_try_again:
                    response = self.arm()
                    if response.success:
                        self.cmd_try_again = False
                
        elif self.fsm_state == State.SET_MISSION:
            if self.sub_mode == "Mission":
                self.fsm_state = State.FLYING_MISSION
            else:
                if self.cmd_try_again:
                    response = self.set_mode(flight_modes.MISSION)
                    if response.success:
                        self.cmd_try_again = False
        
        elif self.fsm_state == State.FLYING_MISSION:
            pass

        elif self.fsm_state == State.REPOSITION:
            pass

        elif self.fsm_state == State.ACTIVE_UPLOAD:
            pass

        elif self.fsm_state == State.PAUSED:
            if self.sub_mode == "Mission":
                self.fsm_state = State.FLYING_MISSION
            elif self.state == "Standby" or not self.armed:
                self.fsm_state = State.GROUNDED
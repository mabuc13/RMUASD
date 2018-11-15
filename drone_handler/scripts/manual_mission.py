import rospy
from enum import Enum
from math import pi, cos, acos, sin, asin, sqrt
from gcs.msg import DronePath, GPS
from telemetry.srv import GotoWaypointRequest
from std_msgs.msg import Int16

ACCEPTANCE_RADIUS = 10
RUN_FREQ = 2

def distGreatCircle(lat1,lon1,lat2,lon2):
    lat1 *= pi/180
    lon1 *= pi/180
    lat2 *= pi/180
    lon2 *= pi/180

    # This calculates the distance in radians
    d = acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lon1-lon2))

    # to convert it to meters we have to first convert from radians to degrees
    # and then translate the degrees to arc-minutes (1/60 degree)
    # and from there to meters 1 arc-minute == 1 nm ~ 1852m

    # for every degree there are 60 arc minutes
    # for every arc minute there are 1852 m
    d *= 180/pi*60*1852

    return d

class State(Enum):
    IDLE = 1
    REPOSITION = 2
    ON_THE_WAY = 3
    LAST_WAYPOINT = 4
    MISSION_DONE = 5

class ManualMission(object):

    def __init__(self, target_sys, target_comp, reposition_handle):
        self.target_sys = 1#target_sys
        self.target_comp = 1#target_comp

        self.mission = []
        self.mission_idx = 0
        self.run_timer = None
        self.reposition_proxy = reposition_handle
        
        self.fsm_state = State.IDLE
        self.cmd_try_again = False

        self.latitude = 0
        self.longitude = 0
        self.relative_alt = 0

        self.main_mode = ""
        self.sub_mode = ""

    def on_cmd_fail(self):
        self.cmd_try_again = False

    def update_mission(self, new_mission):
        self.mission = new_mission
        self.mission_idx = 0

    def update_flight_mode(self, main_mode, sub_mode):
        self.main_mode = main_mode
        self.sub_mode = sub_mode

    def update_position(self, msg):
        self.latitude = msg.lat
        self.longitude = msg.lon
        self.relative_alt = msg.relative_alt

    def start_running(self):
        self.start = True
        self.run_timer = rospy.Timer(rospy.Duration(1/RUN_FREQ), self.run)

    def stop_running(self):
        try:
            self.run_timer.shutdown()
        except Exception as e:
            rospy.logwarn("Can't shut timer down. It isn't running")
            rospy.logwarn(e)

    def run(self, event):
        # this possibly needs to be run after a mission has been uploaded
        # self.set_current_mission_pub.publish(Int16(self.mission_idx))

        rospy.loginfo(self.fsm_state)

        if self.fsm_state == State.IDLE:
            if self.start:
                wp = self.mission[self.mission_idx]
                request = GotoWaypointRequest(
                    relative_alt=True, latitude=wp.latitude,
                    longitude=wp.longitude, altitude=wp.altitude
                    )
                response = self.reposition_proxy(request)

                if response.success:
                    self.fsm_state = State.REPOSITION
                    self.start = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.REPOSITION:
            if self.sub_mode == "Loiter":
                if self.mission_idx < len(self.mission) - 1:
                    self.fsm_state = State.ON_THE_WAY
                else:
                    self.fsm_state = State.LAST_WAYPOINT
            
            elif self.cmd_try_again:
                wp = self.mission[self.mission_idx]
                request = GotoWaypointRequest(
                    relative_alt=True, latitude=wp.latitude,
                    longitude=wp.longitude, altitude=wp.altitude
                    )
                response = self.reposition_proxy(request)
                if response.success:
                    self.cmd_try_again = False

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.ON_THE_WAY:
            next_wp = self.mission[self.mission_idx]
            dist_to_wp = distGreatCircle(self.latitude, self.longitude, next_wp.latitude, next_wp.longitude)
            
            if dist_to_wp < ACCEPTANCE_RADIUS:
                # go to next waypoint
                wp = self.mission[self.mission_idx + 1]
                request = GotoWaypointRequest(
                    relative_alt=True, latitude=wp.latitude,
                    longitude=wp.longitude, altitude=wp.altitude
                    )
                response = self.reposition_proxy(request)

                if response.success:
                    self.fsm_state = State.REPOSITION
                    self.mission_idx += 1

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.LAST_WAYPOINT:
            pass

        # ------------------------------------------------------------------------------ #
        elif self.fsm_state == State.MISSION_DONE:
            pass
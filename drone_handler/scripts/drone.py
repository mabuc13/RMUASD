from collections import deque
from mavlink_lora.msg import mavlink_lora_mission_item_int

# defines 
BUFFER_SIZE = 30

class Drone(object):

    def __init__(self):
        # All variables relating to the status and information of the drone

        self.last_heard = 0

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

        # from mission info
        self.active_waypoint_idx = 0
        self.active_mission_len = 0
        self.active_waypoint = mavlink_lora_mission_item_int()

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
        pass
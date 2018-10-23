#!/usr/bin/env python3

MAVLINK_MSG_ID_HEARTBEAT = 0
MAVLINK_MSG_ID_SET_MODE = 11
MAVLINK_MSG_ID_SET_MODE_LEN = 6
MAVLINK_MSG_ID_RC_CHANNELS_SCALED = 34
MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST = 37
MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST = 38
MAVLINK_MSG_ID_MISSION_ITEM = 39
MAVLINK_MSG_ID_MISSION_REQUEST = 40
MAVLINK_MSG_ID_MISSION_SET_CURRENT = 41
MAVLINK_MSG_ID_MISSION_CURRENT = 42
MAVLINK_MSG_ID_MISSION_COUNT = 44
MAVLINK_MSG_ID_MISSION_CLEAR_ALL = 45
MAVLINK_MSG_ID_MISSION_ITEM_REACHED = 46
MAVLINK_MSG_ID_MISSION_ACK = 47
MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN = 48
MAVLINK_MSG_ID_MISSION_REQUEST_INT = 51
MAVLINK_MSG_ID_MISSION_ITEM_INT = 73
MAVLINK_MSG_ID_COMMAND_LONG = 76
MAVLINK_MSG_ID_COMMAND_LONG_LEN = 33
MAVLINK_MSG_ID_COMMAND_ACK = 77
MAVLINK_MSG_ID_MANUAL_SETPOINT = 81
MAVLINK_MSG_ID_SET_ATTITUDE_TARGET = 82
MAVLINK_MSG_ID_ATTITUDE_TARGET = 83
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED = 84
MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED = 85
MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_LEN = 51
MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT = 86
MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBABL_INT_LEN = 53
MAVLINK_MSG_ID_STATUSTEXT = 253

MAV_CMD_ARM_DISARM = 400
MAV_CMD_MISSION_START = 300
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND_LOCAL = 23
MAV_CMD_NAV_TAKEOFF_LOCAL = 24
MAV_CMD_NAV_LOITER_TO_ALT = 31
MAV_CMD_NAV_GUIDED_ENABLE = 92
MAV_CMD_NAV_DELAY = 93
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_DO_JUMP = 177
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_HOME = 179
MAV_CMD_DO_SET_SERVO = 183

MAV_CMD_NAV_RALLY_POINT = 5100

MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

MAVLINK_MAIN_MODE_LOOKUP = {0: "N/A", 1: "Manual", 2: "Altitude Control", 3: "Position Control",
                            4: "Auto", 5: "Acro", 6: "Offboard", 7: "Stabilized", 8: "Rattitude"}

MAVLINK_SUB_MODE_AUTO_LOOKUP = {0: "", 1: "Ready", 2: "Takeoff", 3: "Loiter", 4: "Mission",
                                5: "Return to Home", 6: "Land", 7: "RTGS", 8: "Follow Me"}

MAVLINK_MAV_TYPE_LOOKUP = { 0: "Generic Micro Air Vehicle", 1: "Fixed Wing Aircraft", 2: "Quadrotor",
                            3: "Coaxial Helicopter", 4: "Normal Helicopter", 5: "Ground Installation",
                            6: "Ground Control Station", 7: "Airship", 8: "Free Balloon", 9: "Rocket",
                            10: "Ground Rover", 11: "Surface Boat", 12: "Submarine", 13: "Hexarotor",
                            14: "Octorotor", 15: "Tricopter", 16: "Flapping Wing", 17: "Kite",
                            18: "Onboard Companion Controller", 19: "Two-rotor VTOL", 20: "Quadrotor VTOL",
                            21: "Tilt-rotor VTOL", 22: "VTOL Reserved 2", 23: "VTOL Reserved 3",
                            24: "VTOL Reserved 4", 25: "VTOL Reserved 5", 26: "Onboard Gimbal",
                            27: "Onboard ADSB Peripheral", 28: "Steerable, Nonrigid Airfoil", 29: "Dodecarotor", 
                            30: "Camera", 31: "Charging Station", 32: "Onboard FLARM Collision Avoidance System"}

MAVLINK_AUTOPILOT_TYPE_LOOKUP = {   0: "Generic", 1: "Reserved", 2: "SLUGS", 3: "ArduPilotMega", 4: "OpenPilot",
                                    5: "Generic Waypoints Only", 6: "Generic Waypoints and Simple Navigation Only",
                                    7: "Generic Mission Full", 8: "Invalid", 9: "PPZ", 10: "UAV Dev Board", 11: "FlexiPilot", 
                                    12: "PX4", 13: "SMACCMPilot", 14: "AutoQuad", 15: "Armazila", 16: "Aerob", 17: "ASLUAV",
                                    18: "SmartAP", 19: "AirRails"}

MAVLINK_MAV_STATE_LOOKUP = {0: "Uninitialized", 1: "Booting Up", 2: "Calibrating", 3: "Standby",
                            4: "Active", 5: "Critical", 6: "Emergency", 7: "Powering Down", 8: "Terminating"}

MAVLINK_SEVERITY_LOOKUP =  {0: "Emergency", 1: "Alert", 2: "Critical", 3: "Error", 4: "Warning",
                            5: "Notice", 6: "Info", 7: "Debug"}

MAV_FRAME_MISSION = 2
MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

SUPPORTED_GLOBAL_FRAMES = [
    MAV_CMD_NAV_WAYPOINT,
    MAV_CMD_NAV_LOITER_UNLIM,
    MAV_CMD_NAV_LOITER_TIME,
    MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_TAKEOFF,
    MAV_CMD_NAV_LOITER_TO_ALT,
    # MAV_CMD_NAV_ROI,
    # MAV_CMD_DO_SET_ROI,
    # MAV_CMD_DO_SET_ROI_LOCATION,
    # MAV_CMD_NAV_VTOL_TAKEOFF,
    # MAV_CMD_NAV_VTOL_LAND,
    # MAV_CMD_NAV_FENCE_RETURN_POINT,
    # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
    # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
    # MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION,
    # MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION,
    MAV_CMD_NAV_RALLY_POINT
]
SUPPORTED_MISSION_FRAMES = [
    MAV_CMD_DO_JUMP,
    # MAV_CMD_NAV_ROI,
    # MAV_CMD_DO_SET_ROI,
    MAV_CMD_DO_CHANGE_SPEED,
    MAV_CMD_DO_SET_HOME,
    MAV_CMD_DO_SET_SERVO,
    # MAV_CMD_DO_LAND_START,
    # MAV_CMD_DO_TRIGGER_CONTROL,
    # MAV_CMD_DO_DIGICAM_CONTROL,
    # MAV_CMD_DO_MOUNT_CONFIGURE,
    # MAV_CMD_DO_MOUNT_CONTROL,
    # MAV_CMD_IMAGE_START_CAPTURE,
    # MAV_CMD_IMAGE_STOP_CAPTURE,
    # MAV_CMD_VIDEO_START_CAPTURE,
    # MAV_CMD_VIDEO_STOP_CAPTURE,
    # MAV_CMD_DO_SET_CAM_TRIGG_DIST,
    # MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL,
    # MAV_CMD_SET_CAMERA_MODE,
    # MAV_CMD_DO_VTOL_TRANSITION,
    MAV_CMD_NAV_DELAY,
    MAV_CMD_NAV_RETURN_TO_LAUNCH,
    # MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET,
    # MAV_CMD_DO_SET_ROI_NONE
]
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

MAVLINK_CMD_ID_ARM_DISARM = 400
MAVLINK_CMD_ID_MISSION_START = 300
MAVLINK_CMD_NAV_RETURN_TO_LAUNCH = 20
MAVLINK_CMD_NAV_LAND = 21
MAVLINK_CMD_NAV_TAKEOFF = 22
MAVLINK_CMD_NAV_LAND_LOCAL = 23
MAVLINK_CMD_NAV_TAKEOFF_LOCAL = 24
MAVLINK_CMD_NAV_GUIDED_ENABLE = 92
MAVLINK_CMD_DO_SET_MODE = 176
MAVLINK_CMD_DO_CHANGE_SPEED = 178

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
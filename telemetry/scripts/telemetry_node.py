#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import struct

from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos, mavlink_lora_status, mavlink_lora_mission_list, mavlink_lora_command_ack
from gcs.msg import * # pylint: disable=W0614
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.srv import SetMode, ChangeSpeed, UploadFromFile, TakeoffDrone, LandDrone
from telemetry.msg import *
from mavlink_defines import * # pylint: disable=W0614
from datetime import datetime
import mission_handler
import command_handler

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
mavlink_lora_status_sub_topic = '/mavlink_status'
mavlink_lora_pos_sub_topic = '/mavlink_pos'
mavlink_lora_atti_sub_topic = '/mavlink_attitude'
mavlink_lora_keypress_sub_topic = '/keypress' 
update_interval = 10



class Telemetry(object):

    def __init__(self):
        self.request_sent = False
        self.first_msg_ok = False

		# status variables
        self.batt_volt = 0.0
        self.last_heard = 0
        self.last_heard_sys_status = 0
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.local_lat = 0.0
        self.local_lon = 0.0
        self.local_alt = -5.0
        self.recorded_sys_id = 0
        self.recorded_comp_id = 0

        self.mission_handler = mission_handler.MissionHandler()
        self.current_mission_no = 0
        self.last_mission_no = 0

        self.command_handler = command_handler.CommandHandler()

        rospy.sleep (1) # wait until everything is running

        self.boot_time = rospy.Time.now().to_sec() * 1000

        # Service handlers
        self.arm_service                = rospy.Service("/telemetry/arm_drone", Trigger, self.command_handler.arm_drone, buff_size=10)
        self.disarm_service             = rospy.Service("/telemetry/disarm_drone", Trigger, self.command_handler.disarm_drone, buff_size=10)
        self.takeoff_service            = rospy.Service("/telemetry/takeoff_drone", TakeoffDrone, self.command_handler.takeoff_drone, buff_size=10)
        self.land_service               = rospy.Service("/telemetry/land_drone", LandDrone, self.command_handler.land_drone, buff_size=10)
        self.start_mission_service      = rospy.Service("/telemetry/start_mission", Trigger, self.command_handler.start_mission, buff_size=10)
        self.return_home_service        = rospy.Service("/telemetry/return_home", Trigger, self.command_handler.return_home, buff_size=10)
        self.set_mode_service           = rospy.Service("/telemetry/set_mode", SetMode, self.command_handler.set_mode, buff_size=10)
        self.guided_enable_service      = rospy.Service("/telemetry/guided_enable", Trigger, self.command_handler.guided_enable, buff_size=10)
        self.guided_disable_service     = rospy.Service("/telemetry/guided_disable", Trigger, self.command_handler.guided_disable, buff_size=10)
        self.change_speed_service       = rospy.Service("/telemetry/change_speed", ChangeSpeed, self.command_handler.change_speed, buff_size=10)
        self.dowload_mission_service    = rospy.Service("/telemetry/download_mission", Trigger, self.mission_handler.download, buff_size=10)
        self.mission_upload_service     = rospy.Service("/telemetry/upload_mission", Trigger, self.mission_handler.upload, buff_size=10)
        self.mission_upload_from_file_service = rospy.Service("/telemetry/upload_mission_from_file", UploadFromFile, self.mission_handler.upload_from_file, buff_size=10)

        # Topic handlers
        self.mavlink_msg_pub        = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        self.statustext_pub         = rospy.Publisher("/telemetry/statustext", telemetry_statustext, queue_size=0)
        self.heartbeat_status_pub   = rospy.Publisher("/telemetry/heartbeat_status", telemetry_heartbeat_status, queue_size=0)
        self.mav_mode_pub           = rospy.Publisher("/telemetry/mav_mode", telemetry_mav_mode, queue_size=0)
        self.vfr_hud_pub            = rospy.Publisher("/telemetry/vfr_hud", telemetry_vfr_hud, queue_size=0)
        self.home_position_pub      = rospy.Publisher("/telemetry/home_position", telemetry_home_position, queue_size=0)
        rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, self.on_mavlink_msg)
        rospy.Subscriber(mavlink_lora_pos_sub_topic, mavlink_lora_pos, self.on_mavlink_lora_pos)
        rospy.Subscriber(mavlink_lora_status_sub_topic, mavlink_lora_status, self.on_mavlink_lora_status)
        rospy.Subscriber(mavlink_lora_keypress_sub_topic, Int8, self.on_keypress)
        rospy.Subscriber("/telemetry/new_mission", mavlink_lora_mission_list, self.mission_handler.on_mission_list)
        rospy.Subscriber("/mavlink_interface/mission/ack", String, self.mission_handler.on_mission_ack)
        rospy.Subscriber("/mavlink_interface/command/ack", mavlink_lora_command_ack, self.command_handler.on_command_ack)
        
        self.rate = rospy.Rate(update_interval)

    def on_mavlink_msg(self,msg):   
        if self.first_msg_ok == False:
            self.first_msg_ok = True
            self.recorded_sys_id = msg.sys_id    
            self.recorded_comp_id = msg.comp_id

        # also send the msg to the mission/command handler for processing
        self.mission_handler.on_mavlink_msg(msg)
        self.command_handler.on_mavlink_msg(msg)

        if msg.msg_id == MAVLINK_MSG_ID_HEARTBEAT:
            (custom_mode, mav_type, autopilot, base_mode, mav_state, mavlink_version) = struct.unpack('<IBBBBB', msg.payload)	
            
            # custom_mode_bytes = custom_mode.to_bytes(4,byteorder='big')
            # (sub_mode, main_mode, _, _) = struct.unpack('<BBBB',bytearray(custom_mode_bytes))             
            sub_mode = custom_mode >> 24
            main_mode = (custom_mode >> 16) & 0xFF

            heartbeat_msg = telemetry_heartbeat_status(
                system_id=msg.sys_id,
                component_id=msg.comp_id,
                timestamp=datetime.now().isoformat(),
                main_mode=MAVLINK_MAIN_MODE_LOOKUP[main_mode],
                sub_mode=MAVLINK_SUB_MODE_AUTO_LOOKUP[sub_mode],
                armed=bool(base_mode & 0x80),
                autopilot=MAVLINK_AUTOPILOT_TYPE_LOOKUP[autopilot],
                base_mode=base_mode,
                mav_state=MAVLINK_MAV_STATE_LOOKUP[mav_state],
                mav_type=MAVLINK_MAV_TYPE_LOOKUP[mav_type],
                mavlink_version=mavlink_version
            )

            mav_mode_msg = telemetry_mav_mode(
                system_id=msg.sys_id,
                component_id=msg.comp_id,
                timestamp=datetime.now().isoformat(),
                armed=bool(base_mode & 0x80),
                manual_input=bool(base_mode & 0x40),
                hil_simulation=bool(base_mode & 0x20),
                stabilized_mode=bool(base_mode & 0x10),
                guided_mode=bool(base_mode & 0x08),
                auto_mode=bool(base_mode & 0x04),
                test_mode=bool(base_mode & 0x02),
                custom_mode=bool(base_mode & 0x01)
            )

            self.heartbeat_status_pub.publish(heartbeat_msg)
            self.mav_mode_pub.publish(mav_mode_msg)

        elif msg.msg_id == MAVLINK_MSG_ID_STATUSTEXT:
            (severity, text) = struct.unpack('<B50s', msg.payload)

            # Convert bytestream to string and remove 0-padding
            text = text.decode("utf-8")
            text = text.rstrip('\0')


            status_msg = telemetry_statustext(
                system_id=msg.sys_id,
                component_id=msg.comp_id,
                timestamp=datetime.now().isoformat(),
                severity_level=severity,
                severity=MAVLINK_SEVERITY_LOOKUP[severity],
                text=text)

            rospy.logwarn(text)
            self.statustext_pub.publish(status_msg)

        elif msg.msg_id == MAVLINK_MSG_ID_VFR_HUD:
            (airspeed, groundspeed, alt, climb, heading, throttle) = struct.unpack('<ffffhH', msg.payload)
            hud_msg = telemetry_vfr_hud(
                system_id=msg.sys_id,
                component_id=msg.comp_id,
                timestamp=datetime.now().isoformat(),
                airspeed=airspeed,
                ground_speed=groundspeed,
                absolute_alt=alt,
                climb_rate=climb,
                heading=heading,
                throttle=throttle
            )

            self.vfr_hud_pub.publish(hud_msg)

        elif msg.msg_id == MAVLINK_MSG_ID_HOME_POSITION:
            (latitude, longitude, altitude, x, y, z, q0, q1, q2, q3, approach_x, approach_y, approach_z) = struct.unpack('<iiifff4ffff', msg.payload)

            home_pos = telemetry_home_position(
                system_id=msg.sys_id,
                component_id=msg.comp_id,
                timestamp=datetime.now().isoformat(),
                latitude=latitude / 1e7,
                longitude=longitude / 1e7,
                altitude=altitude / 1e3,
                x=x,
                y=y,
                z=z
            )

            self.home_position_pub.publish(home_pos)
        # elif msg.msg_id == MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        #     (time_boot_ms, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate, type_mask, coordinate_frame) = struct.unpack('<IfffffffffffHB', msg.payload)

        #     print("x: {}".format(x))
        #     print("y: {}".format(y))
        #     print("z: {}".format(z))
        #     print("frame: {}".format(coordinate_frame))
        #     print("mask: {}".format(type_mask))
        #     pass

        # elif msg.msg_id == MAVLINK_MSG_ID_RC_CHANNELS_SCALED:
        #     print(len(msg.payload))
        #     (time_boot_ms, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, port, rssi) = struct.unpack('<IhhhhhhhhBB', msg.payload)

        #     print("Port: {}".format(port))
        #     print("Channel 1: {}".format(ch1))
        #     print("Channel 2: {}".format(ch2))
        #     print("Channel 3: {}".format(ch3))
        #     print("Channel 4: {}".format(ch4))
        #     print("Channel 5: {}".format(ch5))
        #     print("Channel 6: {}".format(ch6))
        #     print("Channel 7: {}".format(ch7))
        #     print("Channel 8: {}".format(ch8))
        #     print("RSSI: {}".format(rssi))

        # elif msg.msg_id == 36:
        #     # (time_boot_ms, ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, port) = struct.unpack('<IHHHHHHHHB', msg.payload)
        #     pass


    def on_mavlink_lora_pos(self,msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt

    def on_mavlink_lora_status (self, msg):
        self.last_heard = msg.last_heard.secs + msg.last_heard.nsecs/1.0e9
        self.last_heard_sys_status = msg.last_heard_sys_status.secs + msg.last_heard_sys_status.nsecs/1.0e9
        self.batt_volt = msg.batt_volt / 1000.0


    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")


    # def enable_rc_channels(self):
    #     msg = mavlink_lora_msg()

    #     # no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
    #     msg.header.stamp = rospy.Time.now()
    #     msg.msg_id = 66 # request data stream
    #     msg.sys_id = 0
    #     msg.comp_id = 0

    #     # command = 511
    #     # params = (34,500000,0,0,0,0,0)

    #     # self.send_mavlink_msg_id_cmd_long(params,command,1)

    #     # Appears to need to be 1,1
    #     target_sys = 1
    #     target_comp = 1
    #     req_stream_id = 3 # rc channel stream
    #     req_message_rate = 1
    #     start_stop = 0

    #     msg.payload_len = 6
    #     msg.payload = struct.pack('<HBBBB', req_message_rate, target_sys, target_comp, req_stream_id, start_stop)
    #     self.mavlink_msg_pub.publish(msg)

    # def set_gps_global_origin(self,srv):
    #     msg = mavlink_lora_msg()
        
    #     target_sys = 1
    #     latitude = int(srv.lat * 1e7)
    #     longitude = int(srv.lon * 1e7)
    #     altitude = int(self.alt * 1e3)

    #     msg.msg_id = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN
    #     msg.payload = struct.pack('<iiiB', latitude, longitude, altitude, target_sys)
    #     msg.payload_len = 13

    #     self.mavlink_msg_pub.publish(msg)

    #     return OriginResponse()

def main():
    rospy.init_node('telemetry')#, anonymous=True)
    rospy.sleep(1)

    tel = Telemetry()

    rospy.on_shutdown(tel.shutdownHandler)
    # tel.enable_rc_channels()
    # Send global setpoint every 0.4 seconds
    # rospy.Timer(rospy.Duration(0.2),tel.send_local_setpoint)

    rospy.spin()

if __name__ == "__main__":
    main()

'''
def signal_handler(signal,somthing):
    print('You pressed Ctrl+C!')
    sys.exit(0)

def getDroneStatus():
    msg = DroneInfo();
    msg.position.longitude  = 55.6
    msg.position.latitude   = 55.6
    msg.position.altitude   = 100
    msg.next_goal.longitude = 57.6
    msg.next_goal.latitude  = 57.6
    msg.next_goal.altitude  = 100
    msg.velocity[0] = 10.0
    msg.velocity[1] = 1
    msg.velocity[2] = 0.1
    msg.heading = 360 #deg
    msg.battery_SOC = 68 #Percent
    msg.drone_id = 2524362
    msg.GPS_timestamp = 1251351312
    msg.status = 0
    return msg

def new_route_reqest_handler(msg):
    pass

def Drone_State_request_handler(msg):
    pass

DroneStatus_pub = rospy.Publisher('/Telemetry/DroneStatus',DroneInfo, queue_size=10)
RouteRequest_sub = rospy.Subscriber('/gcs/PathRequest', DronePath, new_route_reqest_handler)
DroneState_sub = rospy.Subscriber('/gcs/StateRequest', DroneState, Drone_State_request_handler)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('DroneLinkNode')
    rate = 50
    spins = -1
    while not rospy.is_shutdown():
        spins = spins +1
        rospy.Rate(rate).sleep()

        if spins % rate == 0:
            DroneStatus_pub.publish(getDroneStatus())
'''
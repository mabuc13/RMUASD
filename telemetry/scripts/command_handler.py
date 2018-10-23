#!/usr/bin/env python3

# imports
from sys import argv
import rospy
import rospkg
import time
import struct
from mavlink_defines import * # pylint: disable=W0614
from mavlink_lora.msg import mavlink_lora_msg, mavlink_lora_pos
from telemetry.srv import ChangeSpeed, ChangeSpeedResponse
import command_lib

# defines
MAX_RETRIES = 3
TIMEOUT = 1.5

class CommandHandler(object):

    def __init__(self):
        self.first_msg_ok = False
        self.busy = False
        self.confirmation = 0
        self.retries = 0

        self.lat = 0
        self.lon = 0
        self.alt = 0

        self.cmd_lib = command_lib.command_lib()
        self.sys_id = 0  # reset when receiving first msg
        self.comp_id = 0

        self.mavlink_msg = mavlink_lora_msg()
        self.command_timer = None

        # topic handlers
        self.mavlink_msg_pub        = rospy.Publisher("/mavlink_tx", mavlink_lora_msg, queue_size=0)

        rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_mavlink_lora_pos)

    def on_mavlink_msg(self, msg):
        if self.first_msg_ok == False:
            self.first_msg_ok = True
            self.sys_id = msg.sys_id
            self.cmd_lib.set_target(self.sys_id, self.comp_id)

    def on_mavlink_lora_pos(self,msg):
        self.lat = msg.lat 
        self.lon = msg.lon
        self.alt = msg.alt
       
    def on_command_ack(self, msg):
        self.busy = False
        self.retries = 0
        if self.command_timer != None:
            self.command_timer.shutdown()
        rospy.loginfo(msg.result_text)
        # TODO add parser for what command was acknowledged

    # TODO Create suitable service messages for these calls (maybe include drone ID)
    def arm_drone(self, srv):
        command = MAV_CMD_ARM_DISARM
        params = (1,0,0,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def disarm_drone(self, srv):
        command = MAV_CMD_ARM_DISARM
        params = (0,0,0,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def takeoff_drone(self, srv):
        command = MAV_CMD_NAV_TAKEOFF

        yaw = float('nan') if srv.yaw == -1 else srv.yaw

        if srv.on_the_spot:
            params = (srv.abort_alt,0,0,yaw,self.lat,self.lon, self.alt + srv.alt)
        else:
            params = (srv.abort_alt,0,0,yaw,srv.lat,srv.lon,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def land_drone(self, srv):
        command = MAV_CMD_NAV_LAND

        yaw = float('nan') if srv.yaw == -1 else srv.yaw

        if srv.on_the_spot:
            params = (srv.abort_alt,srv.precision_land,0,yaw,self.lat,self.lon,0)
        else:
            params = (srv.abort_alt,srv.precision_land,0,yaw,srv.lat,srv.lon,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()
        
    def set_mode(self, srv):
        # custom_mode = struct.pack('<BBBB',srv.sub_mode,srv.main_mode,0,0)
        command = MAV_CMD_DO_SET_MODE
        params = (srv.base_mode,srv.main_mode,srv.sub_mode,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def guided_enable(self, srv):
        command = MAV_CMD_NAV_GUIDED_ENABLE
        params = (1,0,0,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def guided_disable(self, srv):
        command = MAV_CMD_NAV_GUIDED_ENABLE
        params = (0,0,0,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def start_mission(self, msg):
        command = MAV_CMD_MISSION_START
        params = (0,0,0,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def return_home(self,msg):
        command = MAV_CMD_NAV_RETURN_TO_LAUNCH
        params = (0,0,0,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def change_speed(self, srv):
        command = MAV_CMD_DO_CHANGE_SPEED
        # params = (srv.speed_type,srv.speed,srv.throttle,srv.abs_rel,0,0,0)
        params = (1,srv.speed,-1,0,0,0,0)

        self.cmd_lib.pack_command_long(params, command, self.confirmation)

        return self.send_mavlink_msg()

    def send_mavlink_msg(self):
        if self.busy:
            rospy.logwarn("Can't execute command. Handler is busy.")
            success = False
            message = "Can't execute command. Handler is busy." 
        else:
            self.busy = True
            self.mavlink_msg.header.stamp = rospy.Time.now()
            self.mavlink_msg = self.cmd_lib.msg
            self.mavlink_msg_pub.publish(self.mavlink_msg)
            success = True
            message = "Command sent"
            rospy.loginfo(message)

            # start timeout
            self.command_timer = rospy.Timer(
                period=rospy.Duration(TIMEOUT),
                callback=self.command_timeout,
                oneshot=True
            )
        
        return success, message

    def command_timeout(self, event):
        self.retries += 1
        rospy.loginfo("Command timeout!")

        if self.retries > MAX_RETRIES:
            self.retries = 0
            self.confirmation = 0
            rospy.logwarn("Too many failures - giving up.")
            self.busy = False
        else:
            rospy.loginfo("Sending command again")
            self.confirmation += 1
            self.mavlink_msg_pub.publish(self.mavlink_msg)
            
            # start timeout
            self.command_timer = rospy.Timer(
                period=rospy.Duration(TIMEOUT),
                callback=self.command_timeout,
                oneshot=True
            )
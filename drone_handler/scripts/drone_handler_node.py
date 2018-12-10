#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import struct
from datetime import datetime
import drone
import manual_mission

from gcs.msg import * # pylint: disable=W0614
from std_msgs.msg import Int8, String, Time
from std_srvs.srv import Trigger, TriggerResponse
from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import mavlink_lora_attitude, mavlink_lora_pos, mavlink_lora_status, mavlink_lora_mission_ack, mavlink_lora_command_ack
from gcs.msg import DroneInfo, GPS, NiceInfo
from node_monitor.msg import heartbeat

# defines
INFO_FREQ = 10

# parameters
update_interval = 10

class DroneHandler(object):

    def __init__(self):

        # hardcode the drone ID to 1 which is the drone we will be using
        self.drones = {1: drone.Drone()}

		# status variables
        rospy.sleep (1) # wait until everything is running

        self.run_ready = True
        self.info_ready = True

        # Service handlers
        self.mission_request_service = rospy.Service("/drone_handler/mission_request", Trigger, self.mission_request, buff_size=10)

        # Topic handlers
        # self.mavlink_msg_pub        = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0)
        self.drone_info_pub     = rospy.Publisher("/drone_handler/DroneInfo", DroneInfo, queue_size=0)
        self.nice_info_pub      = rospy.Publisher("/drone_handler/NiceInfo", NiceInfo, queue_size=0)
        self.heartbeat_main_pub = rospy.Publisher("/drone_handler/heartbeat/main", Time, queue_size=0)
        self.heartbeat_run_pub  = rospy.Publisher("/drone_handler/heartbeat/run", Time, queue_size=0)
        self.heartbeat_info_pub = rospy.Publisher("/drone_handler/heartbeat/info", Time, queue_size=0)

        rospy.Subscriber("/gcs/forwardPath", DronePath, self.on_drone_path)
        rospy.Subscriber("/gcs/moveTo", moveTo, self.on_move)
        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/telemetry/mav_mode", telemetry_mav_mode, self.on_mav_mode)
        rospy.Subscriber("/telemetry/statustext", telemetry_statustext, self.on_statustext)
        rospy.Subscriber("/telemetry/mission_info", telemetry_mission_info, self.on_mission_info)
        rospy.Subscriber("/telemetry/vfr_hud", telemetry_vfr_hud, self.on_vfr_hud)
        rospy.Subscriber("/telemetry/home_position", telemetry_home_position, self.on_home_position)
        rospy.Subscriber("/telemetry/cmd_retry_fail", telemetry_cmd_retry_fail, self.on_cmd_fail)
        rospy.Subscriber("/mavlink_interface/mission/ack", mavlink_lora_mission_ack, self.on_mission_ack)
        rospy.Subscriber("/mavlink_interface/command/ack", mavlink_lora_command_ack, self.on_command_ack)
        rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_drone_pos)
        rospy.Subscriber("/mavlink_attitude", mavlink_lora_attitude, self.on_drone_attitude)
        rospy.Subscriber("/mavlink_status", mavlink_lora_status, self.on_drone_status)

        # rospy.Timer(rospy.Duration(0.1), self.mission_info_cb)

        self.rate = rospy.Rate(update_interval)

    def on_move(self, msg):
        if msg.drone_id in self.drones:
            drone = self.drones[msg.drone_id]
            drone.on_move(msg)

    def on_cmd_fail(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]
            drone.on_cmd_fail(msg)

    def on_drone_path(self, msg):
        if msg.DroneID in self.drones:
            drone = self.drones[msg.DroneID]

            drone.mission_id = msg.PathID
            drone.update_mission(msg.Path)
            drone.start_mission(msg.loiterAtEnd)
            print("PATH LENGTH: ", len(msg.Path))
        
    def on_heartbeat_status(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.main_mode     = msg.main_mode
            drone.sub_mode      = msg.sub_mode
            drone.autopilot     = msg.autopilot
            drone.type          = msg.mav_type
            drone.state         = msg.mav_state

            drone.manual_mission.update_flight_mode(msg.main_mode, msg.sub_mode)

    def on_mav_mode(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.armed             = msg.armed
            drone.manual_input      = msg.manual_input
            drone.hil_simulation    = msg.hil_simulation
            drone.stabilized_mode   = msg.stabilized_mode
            drone.guided_mode       = msg.guided_mode
            drone.auto_mode         = msg.auto_mode
            drone.test_mode         = msg.test_mode
            drone.custom_mode       = msg.custom_mode

    def on_statustext(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.statustext.append(msg.text)
            drone.severity.append(msg.severity)

    def on_vfr_hud(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.ground_speed = msg.ground_speed
            drone.climb_rate = msg.climb_rate
            drone.throttle = msg.throttle

    def on_home_position(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.home_position = GPS(msg.latitude, msg.longitude, msg.altitude)
            # drone.home_position_local = local_pos(msg.x, msg.y, msg.z)

    def on_mission_info(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            if drone.manual_mission.fsm_state == manual_mission.State.IDLE:
                drone.active_mission_idx = msg.active_waypoint_idx
                drone.active_mission_len = msg.active_mission_len
                # drone.active_sub_waypoint       = msg.current_item
            else:
                # print("Manual idx: {}".format(drone.manual_mission.mission_idx))
                drone.active_mission_idx = drone.manual_mission.mission_idx     
                drone.active_mission_len = len(drone.manual_mission.mission)  


    def on_drone_attitude(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.up_time = msg.time_usec / 1e6
            drone.roll = msg.roll
            drone.pitch = msg.pitch
            drone.yaw = msg.yaw
            # drone.last_heard = msg.header.stamp

    def on_drone_status(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            drone.battery_voltage = msg.batt_volt / 1000.0
            drone.battery_SOC = msg.batt_remaining
            drone.cpu_load = msg.cpu_load
            drone.msg_sent_gcs = msg.msg_sent_gcs
            drone.msg_received_gcs = msg.msg_received_gcs
            drone.msg_dropped_gcs = msg.msg_dropped_gcs
            drone.msg_dropped_uas = msg.msg_dropped_uas
            drone.last_heard = msg.last_heard

    def on_drone_pos(self, msg):
        if msg.system_id in self.drones:
            drone = self.drones[msg.system_id]

            if not drone.active:
                drone.active = True
            gps_timestamp = int(msg.time_usec / 1e6)

            # check if drone sends out boot time instead of epoch time
            if gps_timestamp < 1e9:
                gps_timestamp = int(time.time())

            drone.gps_timestamp = gps_timestamp
            drone.up_time = int(msg.time_boot_usec / 1e6)
            drone.latitude = msg.lat
            drone.longitude = msg.lon
            drone.absolute_alt = msg.alt
            drone.relative_alt = msg.relative_alt
            drone.heading = msg.heading
            # timestamp = datetime.utcfromtimestamp(msg.time_usec / 1e6).strftime('%Y-%m-%d %H:%M:%S')
            # print(timestamp)
            drone.calc_remaining_distance()
            drone.manual_mission.update_position(msg)

    def on_mission_ack(self, msg):
        if msg.drone_id in self.drones:
            drone = self.drones[msg.drone_id]
            drone.on_mission_ack(msg)

    def on_command_ack(self, msg):
        if msg.drone_id in self.drones:
            drone = self.drones[msg.drone_id]
            drone.on_command_ack(msg)

    def send_info_cb(self, event):
        # TODO iterate over all drones
        drone = self.drones[1]

        if drone.active:

            try:
                need2know = DroneInfo(
                    drone_id=drone.id,
                    position=GPS(drone.latitude, drone.longitude, drone.relative_alt),
                    next_waypoint=drone.active_waypoint_gps,
                    armed=drone.armed,
                    ground_speed=drone.ground_speed,
                    ground_speed_setpoint=5,
                    heading=drone.heading,
                    battery_voltage=drone.battery_voltage,
                    battery_SOC=drone.battery_SOC,
                    relative_alt=drone.relative_alt,
                    absolute_alt=drone.absolute_alt,
                    GPS_timestamp=drone.gps_timestamp,
                    status=drone.gcs_status,
                    path_id=drone.mission_id,
                    mission_index=drone.active_mission_idx,
                    mission_length=drone.active_mission_len
                )

                nice2know = NiceInfo(
                    drone_id=drone.id,
                    drone_handler_state=str(drone.fsm_state),
                    last_heard=drone.last_heard, 
                    up_time=int(drone.up_time),
                    RPY=[drone.roll, drone.pitch, drone.yaw],
                    main_flightmode=drone.main_mode,
                    sub_flightmode=drone.sub_mode,
                    msg_sent_gcs=drone.msg_sent_gcs,
                    msg_received_gcs=drone.msg_received_gcs,
                    msg_dropped_gcs=drone.msg_dropped_gcs,
                    msg_dropped_uas=drone.msg_dropped_uas,
                    active_waypoint_idx=drone.active_mission_idx,
                    active_mission_len=drone.active_mission_len,
                    armed=drone.armed,
                    manual_input=drone.manual_input,
                    hil_simulation=drone.hil_simulation,
                    stabilized_mode=drone.stabilized_mode,
                    guided_mode=drone.guided_mode,
                    auto_mode=drone.auto_mode,
                    test_mode=drone.test_mode,
                    custom_mode=drone.custom_mode,
                    autopilot=drone.autopilot,
                    mav_state=drone.state,
                    mav_type=drone.type,
                    climb_rate=drone.climb_rate,
                    throttle=drone.throttle,
                    cpu_load=drone.cpu_load,
                    home=drone.home_position
                )

                self.drone_info_pub.publish(need2know)
                self.nice_info_pub.publish(nice2know)
            except Exception as e:
                print(e)


    def mission_request(self, srv):
        # TODO implement for all drones
        drone = self.drones[1]
        drone.start_mission()

        return TriggerResponse()

    def run_cb(self, event):
        # TODO implement for all drones
        drone = self.drones[1]
        drone.run()

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

if __name__ == "__main__":
    rospy.init_node('drone_handler')#, anonymous=True)
    rospy.sleep(1)

    dh = DroneHandler()

    # This is bad because exceptions in the callbacks will just kill the timers and not the node
    rospy.Timer(rospy.Duration(1/INFO_FREQ), dh.send_info_cb)
    rospy.Timer(rospy.Duration(1/2), dh.run_cb)

    rospy.on_shutdown(dh.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'drone_handler'
    heart_msg.rate = update_interval
        
    while not rospy.is_shutdown():

        # # this makes sure that both function calls is run in the same thread, so it is easier to detect exceptions
        # if dh.info_ready:
        #     dh.send_info()
        #     dh.info_ready = False

        # if dh.run_ready:
        #     dh.run()
        #     dh.run_ready = False
        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        dh.rate.sleep()

#!/usr/bin/env python3
import rospy
import time
import wiringpi as wp

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from node_monitor.msg import heartbeat

# defines
BLUE = 22
GREEN = 10
YELLOW = 9
RED = 11
LED_PINS = [22, 10, 9, 11]

# parameters
update_interval = 10

class PiMonitor(object): 

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        self.blue_status = 0
        self.green_status = 0
        self.yellow_status = 0
        self.red_status = 0

        wp.wiringPiSetupGpio()
        for pin in LED_PINS:
            wp.pinMode(pin, 1)

        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
        rospy.Subscriber("/telemetry/set_landing_target", telemetry_landing_target, self.on_landing_target)
        rospy.Subscriber("/telemetry/imu_data_ned", telemetry_imu_ned, self.on_imu_data)

        rospy.Subscriber("/mavlink_pos", mavlink_lora_pos, self.on_drone_pos)
        rospy.Subscriber("/mavlink_attitude", mavlink_lora_attitude, self.on_drone_attitude)
        #rospy.Subscriber("/mavlink_status", mavlink_lora_status, self.on_drone_status)

    def on_imu_data(self, msg):
        pass

    def on_heartbeat_status(self, msg):
        wp.digitalWrite(RED, self.red_status)
        self.red_status ^= 1

    def on_landing_target(self, msg):
        wp.digitalWrite(YELLOW, self.yellow_status)
        self.yellow_status ^= 1

    def on_drone_pos(self, msg):
        wp.digitalWrite(BLUE, self.blue_status)
        self.blue_status ^= 1

    def on_drone_attitude(self, msg):
        wp.digitalWrite(GREEN, self.green_status)
        self.green_status ^= 1

    def on_drone_status(self, msg):
        wp.digitalWrite(BLUE, self.blue_status)
        self.blue_status ^= 1

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

if __name__ == "__main__":
    rospy.init_node('precision_landing')
    rospy.sleep(1)

    pm = PiMonitor()

    rospy.on_shutdown(pm.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'precision_landing'
    heart_msg.rate = update_interval
        
    while not rospy.is_shutdown():
        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        pm.rate.sleep()

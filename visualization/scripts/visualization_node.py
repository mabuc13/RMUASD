#!/usr/bin/env python3
import rospy
import time

from telemetry.msg import * # pylint: disable=W0614
from mavlink_lora.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from precision_landing.msg import precland_sensor_data
from node_monitor.msg import heartbeat

# defines

# parameters
update_interval = 10


class Visualization(object): 

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        rospy.Subscriber("/telemetry/heartbeat_status", telemetry_heartbeat_status, self.on_heartbeat_status)
    

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

    def on_heartbeat_status(self, msg):
        pass

    def run(self):
        pass


if __name__ == "__main__":
    rospy.init_node('visualization_node')
    rospy.sleep(1)

    v = Visualization()

    rospy.on_shutdown(v.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'precision_landing'
    heart_msg.rate = update_interval

    while not rospy.is_shutdown():
        v.run()

        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        v.rate.sleep()

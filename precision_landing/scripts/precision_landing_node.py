#!/usr/bin/env python3
import rospy
import time
import struct
from smbus2 import SMBus

from telemetry.msg import * # pylint: disable=W0614
from geometry_msgs.msg import Point
from node_monitor.msg import heartbeat

# defines
TAG_ADDRESS = 13

LANDING_TARGET_REF = 0

# 3 floats amount to a length of 12 bytes
LANDING_TARGET_SIZE = 12

# parameters
update_interval = 10

class PrecisionLanding(object): 

    def __init__(self):
        self.rate = rospy.Rate(update_interval)

        self.bus = SMBus(1)
        self.landing_target = Point()


        self.landing_target_pub = rospy.Publisher("/telemetry/set_landing_target", telemetry_landing_target, queue_size=0)

    def get_landing_target(self):
        try:
            data = self.bus.read_i2c_block_data(TAG_ADDRESS, LANDING_TARGET_REF, LANDING_TARGET_SIZE)
            (x,y,z) = struct.unpack('<fff',bytearray(data))
            self.landing_target = Point(x, y, z)
        except Exception as e:
            rospy.logwarn(e)

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

if __name__ == "__main__":
    rospy.init_node('precision_landing')
    rospy.sleep(1)

    pl = PrecisionLanding()

    rospy.on_shutdown(pl.shutdownHandler)
    # rospy.spin()

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'precision_landing'
    heart_msg.rate = update_interval
        
    while not rospy.is_shutdown():

        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)
        pl.rate.sleep()

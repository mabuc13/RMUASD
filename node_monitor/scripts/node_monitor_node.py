#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import rosnode
from node_monitor.msg import *


def signal_handler(signal,somthing):
    print('[Internet node]: You pressed Ctrl+C!')
    rospy.signal_shutdown('ctrl+c')
    sys.exit(0)

class NodeTime(dict):
    def __missing__(self, key):
        return 0

class NodeMonitor(object):

    def __init__(self):

		# status variables
        rospy.sleep (1) # wait until everything is running
        self.nodeTimes = NodeTime()
        self.nodeRates = NodeTime()
        self.nodeRates['node_monitor'] = 10
        self.nodeState = NodeTime()
        self.nodeState['node_monitor'] = heartbeat.nothing
        #subscriber
        rospy.Subscriber("/node_monitor/input/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/gcs/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/utm_parser/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/pathplan/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/telemtry/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/drone_handler/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/internet/Heartbeat", heartbeat, self.handler_heartbeat)
        #rospy.Subscriber("/remot3/Heartbeat", heartbeat, self.handler_heartbeat)

        self.Heartbeat_pub = rospy.Publisher('/node_monitor/Heartbeat', heartbeat, queue_size = 10)
        self.NodeSates_pub = rospy.Publisher('/node_monitor/node_list',nodeOkList,queue_size= 10)

        self.rate = rospy.Rate(self.nodeRates['node_monitor'])
        self.heartbeat = heartbeat()
        self.heartbeat.header.frame_id = "node_monitor"
        self.heartbeat.rate = 10

    def handler_heartbeat(self,msg):
        if msg.rate == 0:
            msg.rate = 1
        self.nodeTimes[msg.header.frame_id] = msg.header.stamp.to_sec();
        self.nodeRates[msg.header.frame_id] = msg.rate
        self.nodeState[msg.header.frame_id] = msg.severity

        if not len(msg.text) == 0:
            text = "["+msg.header.frame_id+"]["
            if msg.severity == msg.info:
                text = text+"info]"
            elif msg.severity == msg.warning:
                text = text+"warning]"
            elif msg.severity == msg.error:
                text = text+"error]"
            elif msg.severity == msg.critical_error:
                text = text+"critical]"
            elif msg.severity == msg.fatal_error:
                text = text+"fatal]"

            text = text+"["+ str(msg.header.stamp.sec) + "]: " + msg.text

            msgO = self.heartbeat
            msgO.text = text
            msgO.severity = msg.severity
            self.Heartbeat_pub.publish(msgO)

    def shutdownHandler(self):
        # shutdown services
        print("Shutting down")

    def run(self):
        runs = 0
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.nodeTimes['node_monitor'] = rospy.Time.now().to_sec()
            nodeStatusList = nodeOkList()
            for key, value in self.nodeTimes.items():
                nodeStatus = nodeOk()
                nodeStatus.name = str(key)
                nodeStatus.nodeState = self.nodeState[key]

                print(str(key)+": " + str(rospy.Time.now().to_sec() - ((1/self.nodeRates[key])*2 + value)) )
                if rospy.Time.now().to_sec() > ((1/self.nodeRates[key])*4 + value):
                    #print("none responsive")
                    nodeStatus.ok = nodeOk.none_responsive
                elif rospy.Time.now().to_sec() > ((1/self.nodeRates[key])*2 + value):
                    #print("late")
                    nodeStatus.ok = nodeOk.late
                else:
                    #print("fine")
                    nodeStatus.ok = nodeOk.fine

                nodeStatusList.Nodes.append(nodeStatus)
                    
            self.NodeSates_pub.publish(nodeStatusList)
                

                

            if runs == 10:
                self.Heartbeat_pub.publish(self.heartbeat)
                runs = 0

            runs = runs +1




if __name__ == "__main__":
    
    rospy.init_node('node_monitor')#, anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    nm = NodeMonitor()

    rospy.on_shutdown(nm.shutdownHandler)

    nm.run()

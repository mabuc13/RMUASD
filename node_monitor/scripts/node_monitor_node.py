#!/usr/bin/env python3

import signal
import sys
import rospy
import time
import rosnode
from drone_decon.msg import *
from node_monitor.msg import *
from mavlink_lora.msg import mavlink_lora_status
import threading


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
        rospy.sleep (3) # wait until everything is running
        self.dataLock = threading.Lock()
        self.nodeTimes = NodeTime()
        self.nodeRates = NodeTime()
        self.nodeRates['node_monitor'] = 10
        self.nodeState = NodeTime()
        self.nodeState['node_monitor'] = heartbeat.nothing
        #subscriber
        rospy.Subscriber("/node_monitor/input/Heartbeat", heartbeat, self.handler_heartbeat)
        rospy.Subscriber("/mavlink_status",mavlink_lora_status,self.handler_mavlink_lora_heartbeat)
        rospy.Subscriber("/drone_decon/Heartbeat",heartbeatDecon,self.handler_heartbeat_decon)

        self.Heartbeat_pub = rospy.Publisher('/node_monitor/Heartbeat', heartbeat, queue_size = 10)
        self.NodeSates_pub = rospy.Publisher('/node_monitor/node_list',nodeOkList,queue_size= 10)

        self.rate = rospy.Rate(self.nodeRates['node_monitor'])
        self.heartbeat = heartbeat()
        self.heartbeat.header.frame_id = "node_monitor"
        self.heartbeat.rate = 10
        

        self.first_send()
    def first_send(self):
        msg = nodeOkList()
        
        Nodelist = ['gcs','node_monitor','drone_handler','telemetry','utm_parser','mavlink','pathplan','internet','remot3','collision_detector','drone_decon']
        Nodelist.sort()
        for text in Nodelist:
            tmp = nodeOk()
            tmp.ok = 0
            tmp.nodeState = 0
            tmp.name = text
            msg.Nodes.append(tmp)

        for i in range(10): #first sending sometime is ignored
            self.NodeSates_pub.publish(msg)
            self.rate.sleep()

    def handler_heartbeat_decon(self,msg):
        newMsg = heartbeat()
        newMsg.header = msg.header
        newMsg.rate = msg.rate
        newMsg.text = msg.text
        newMsg.severity = msg.severity
        self.handler_heartbeat(newMsg)

    def handler_mavlink_lora_heartbeat(self,msg):
        newMsg = heartbeat()
        newMsg.header = msg.header
        newMsg.rate = 2
        newMsg.header.frame_id = "mavlink"
        self.handler_heartbeat(newMsg)

    def handler_heartbeat(self,msg):
        if msg.rate == 0:
            msg.rate = 1
        with self.dataLock:
            self.nodeTimes[msg.header.frame_id] = msg.header.stamp.to_sec();
            self.nodeRates[msg.header.frame_id] = msg.rate
            self.nodeState[msg.header.frame_id] = msg.severity

        if not len(msg.text) == 0:
            text = "["+msg.header.frame_id+"]: "

            text = text+ msg.text

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
            with self.dataLock:
                for key, value in self.nodeTimes.items():
                    nodeStatus = nodeOk()
                    nodeStatus.name = str(key)
                    nodeStatus.nodeState = self.nodeState[key]

                    #print(str(key)+": " + str(rospy.Time.now().to_sec() - ((1/self.nodeRates[key])*2 + value)) )
                    if rospy.Time.now().to_sec() > ((1/self.nodeRates[key])*4 + value):
                        #print(str(key)+": none responsive")
                        nodeStatus.ok = nodeOk.none_responsive
                    elif rospy.Time.now().to_sec() > ((1/self.nodeRates[key])*2 + value):
                        #print(str(key)+": late")
                        nodeStatus.ok = nodeOk.late
                    else:
                        #print("fine")
                        nodeStatus.ok = nodeOk.fine

                    nodeStatusList.Nodes.append(nodeStatus)
                        
            self.NodeSates_pub.publish(nodeStatusList)
                

                

            if runs == 10:
                self.heartbeat.rate = self.nodeRates["node_monitor"]/10
                self.heartbeat.header.stamp = rospy.Time.now()
                self.Heartbeat_pub.publish(self.heartbeat)
                runs = 0

            runs = runs +1




if __name__ == "__main__":
    
    rospy.init_node('node_monitor')#, anonymous=True)
    signal.signal(signal.SIGINT, signal_handler)
    nm = NodeMonitor()

    rospy.on_shutdown(nm.shutdownHandler)

    nm.run()

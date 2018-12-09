#!/usr/bin/env python3
import socket
import signal
import sys
import rospy
import time
from _thread import *
import threading

from std_msgs.msg import String
from node_monitor.msg import *

def signal_handler(signal,somthing):
    print('[Internet node]: You pressed Ctrl+C!')
    node.s.close()
    sys.exit(0)



heart_msg = None

class ROSserver(object):
    def __init__(self):
        rospy.init_node('InternetNode')
        self.pub = rospy.Publisher('/internet/FromInternet',String, queue_size=10)
        self.sub = rospy.Subscriber('/internet/ToInternet', String, self.toInternet)
        self.senderLock = threading.Lock()
        self.printLock = threading.Lock()
        self.server = "localhost"
        self.port = 5555
        self.Ready2Send = False
        self.connectInternet("msg=HandShake")
    def connectInternet(self,msg): # Establish connection to internet
        self.Ready2Send = False
        self.toSend = [msg]
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #Specify connection Type
        try:
            with self.printLock:
                print("[Internet node]: Connecting to: "+self.server+":"+str(self.port))
            self.s.connect((self.server,self.port))
            self.Ready2Send = True
        except socket.error as e:
            with self.printLock:
                print("[Internet node]: "+str(e))
    def toInternet(self,data):     # Read msg heading for the internet and check if New IP address is specifyd
        with self.printLock:
            #print("[Internet node]: "+data.data)
            pass
        data=str(data.data)
        data.replace(" ","")
        serPortChanged = False
        Ser = self.valueOf(data,"server")
        if not Ser == "NULL":
            self.server = Ser
            serPortChanged = True
            data.replace(",server="+Ser,"")

        Ser = self.valueOf(data,"port")
        if not Ser == "NULL":
            self.port = int(Ser)
            serPortChanged = True
            data.replace(",port="+Ser,"")

        if serPortChanged:
            with self.senderLock:
                self.s.close()
                self.connectInternet(data);
        else:
            self.toSend.append(data)
    def sendPck(self):             # Check if any new messages are available and send them of
        if len(self.toSend) >0:
            with self.printLock:
                # print("[Internet node]: "+self.toSend[0])
                pass
            with self.senderLock:
                self.s.send(self.toSend.pop(0).encode())
        else:
            time.sleep(1)
    def recvPck(self):             # Wait for package to be recived
        self.fromInternet(self.s.recv(4096))
    def fromInternet(self,data):   # Format the data from the internet and publish to rostopic
        data.decode()
        data = str(data)
        data = data[2:len(data)-1]
        global heart_msg
        if (not (self.valueOf(data,'name') =='Server')) and len(data) > 5:
            with self.printLock:
                print("[Internet node]: "+"Data recived: "+data) 
            self.pub.publish(data)
        elif self.valueOf(data,'name') =='Server' and not heart_msg == None:
            if self.valueOf(data,'succes')=='1':
                heart_msg.severity = heartbeat.nothing
                heart_msg.text = ""
            elif self.valueOf(data,'succes')=='0':
                heart_msg.severity = heartbeat.warning
                heart_msg.text = self.valueOf(data,'msg')

    def valueOf(self,msg,value):
        text = msg.split(',')
        ret = 'NULL'
        for t in text:
            t.replace(" ","")
            if value in t:
                ret = t.split('=')
                ret = ret[1]
        return ret

node = ROSserver()

def sender(void):
    while True:
        try:
            node.connectInternet("name=gcs")
            while True:
                if node.Ready2Send:
                    node.sendPck()
                else:
                    time.sleep(1)
        except socket.error as e:
            with node.printLock:
                print("[Internet node]: Sending Failed")
                print("[Internet node]: "+str(e))

RecErrNoLast = -1
def reciver(void):
    global RecErrNoLast
    while True:
        try:
            node.connectInternet("name=gcs")
            while True:
                if node.Ready2Send:
                    node.recvPck()
                else:
                    time.sleep(1)
        except socket.error as e:
            if not e.errno == RecErrNoLast:
                RecErrNoLast = e.errno
                with node.printLock:
                    print("[Internet node]: Recived Failed")
                    print("[Internet node]: "+str(e))

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    heart_msg = heartbeat()
    start_new_thread(sender,(1,))
    start_new_thread(reciver,(1,))

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    
    heart_msg.header.frame_id = 'internet'
    heart_msg.rate = 1

    while not rospy.is_shutdown():
        rospy.Rate(heart_msg.rate).sleep()
        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)

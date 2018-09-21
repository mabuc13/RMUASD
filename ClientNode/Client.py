#!/usr/bin/env python3
import socket
import signal
import sys
import rospy
import time
from _thread import *
import threading

from std_msgs.msg import String

def signal_handler(signal,somthing):
    print('You pressed Ctrl+C!')
    node.s.close()
    sys.exit(0)

class ROSserver(object):
    def __init__(self):
        rospy.init_node('InternetNode')
        self.pub = rospy.Publisher('FromInternet',String, queue_size=10)
        self.sub = rospy.Subscriber('ToInternet', String, self.toInternet)
        self.senderLock = threading.Lock()
        self.server = "localhost"
        self.port = 5555
        self.connectInternet()
    def connectInternet(self):
        self.toSend = ["msg=HandShake"]
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            print("Connecting to: "+self.server+":"+str(self.port))
            self.s.connect((self.server,self.port))
        except socket.error as e:
            print(str(e))
    def toInternet(self,data):
        data=str(data)
        data.replace(" ","")
        serPortChanged = False
        Ser = self.valueOf(data,"server")
        if not Ser == "NULL":
            self.server = Ser
            serPortChanged = True
            data.replace(",server="+Ser,"")

        Ser = self.valueOf(data,"port")
        if not Ser == "NULL":
            Ser = Ser[:len(Ser)-1]
            self.port = int(Ser)
            serPortChanged = True
            data.replace(",port="+Ser,"")

        if serPortChanged:
            print("Server addr: "+ self.server+":"+str(self.port))
            with self.senderLock:
                self.s.close()
                self.connectInternet();
        self.toSend.append(data)
    def sendPck(self):
        if len(self.toSend) >0:
            with self.senderLock:
                self.s.send(self.toSend.pop(0).encode())
    def recvPck(self):
        self.fromInternet(self.s.recv(4096))
    def fromInternet(self,data):
        data.decode()
        print(data)
        self.pub.publish(data)
    def valueOf(self,msg,value):
        text = msg.split(',')
        ret = 'NULL'
        for t in text:
                if value in t:
                  ret = t.split('=')
                  ret = ret[1]
        return ret

node = ROSserver()

def sender(void):
    try:
        while True:
            node.sendPck()
    except socket.error as e:
        print("Sending Failed")
        print(str(e))
def reciver(void):
    try:
        while True:
            node.recvPck()
    except socket.error as e:
        print("Recived Failed")
        print(str(e))

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    start_new_thread(sender,(1,))
    start_new_thread(reciver,(1,))
    while True:
        rospy.Rate(30).sleep()

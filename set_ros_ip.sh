#!/bin/sh

wInterface=$(route | grep '^default' | grep -o '[^ ]*$')
wlanIP=$(ifconfig $wInterface | grep "inet addr:" | cut -f2 -d: | cut -f1 -d' ')

rosMaster=10.42.0.1:11311

export ROS_IP=$wlanIP
export ROS_MASTER_URI=http://$rosMaster

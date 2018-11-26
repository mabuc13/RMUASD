#!/usr/bin/env python3
import requests
import httplib2
import json
import rospy
from urllib.request import urlopen
from json import dumps
from internet.srv import *
import socket
from node_monitor.msg import *

#"RUI2QzlDRjQtMTg0NC00RjU0LUE0NDItMDIwNDJGRDBGOERE"
def getToken(developerKey,username,password):
    url = "https://api.remot3.it/apv/v23.5/user/login"
    print ("[remot3]: Connecting to: "+ url)
    payload = "{ \"username\" : \""+username+"\", \"password\" : \""+password+"\" }"
    headers = {
        'developerkey': developerKey,
        'content-type': "application/json",
        'cache-control': "no-cache"
        }

    response = requests.request("POST", url, data=payload, headers=headers)
    jsonFile = response.json()
    print("[remot3]: Found token: "+ jsonFile['token'])
    return jsonFile['token']

def getDevice(token,developerKey):
    apiMethod="https://"
    apiServer="api.remot3.it"
    apiVersion= "/apv/v23.5"
    # add the token here which you got from the /user/login API call
    deviceListURL = apiMethod + apiServer + apiVersion + "/device/list/all"
    content_type_header     = "application/json"

    deviceListHeaders = {
                    'Content-Type': content_type_header,
                    'developerkey': developerKey,
      							# you need to get token from a call to /user/login
                    'token': token,
                }


    httplib2.debuglevel     = 0
    http                    = httplib2.Http()

    print ("[remot3]: Connecting to: " + deviceListURL)
    response, content = http.request( deviceListURL,
                                          'GET',
                                          headers=deviceListHeaders)


    deviceJson = json.loads(content.decode())
    print("[remot3]: Device found: " + deviceJson['status'])
    #print(deviceJson['devices'])
    #print(content)
    return deviceJson['devices']

def getService(name,device):
    for service in device:

        if service['devicealias'] == name:
            print ("[remot3]: service found: "+ service['devicealias'])
            return service['deviceaddress']
    return 'NULL'

def proxyConnect(UID, token,developerKey):
    httplib2.debuglevel     = 0
    http                    = httplib2.Http()
    content_type_header     = "application/json"

    apiMethod="https://"
    apiServer="api.remot3.it"
    apiVersion="/apv/v23.5"

  # this is equivalent to "whatismyip.com"
  # in the event your router or firewall reports a malware alert
  # replace this expression with your external IP as given by
  # whatismyip.com

    my_ip = urlopen('http://ip.42.pl/raw').read().decode()
    print("[remot3]: my Ip: " + my_ip)

    proxyConnectURL = apiMethod + apiServer + apiVersion + "/device/connect"

    proxyHeaders = {
                'Content-Type': content_type_header,
                'developerkey': developerKey,
                'token': token
            }

    proxyBody = {
                'deviceaddress': UID,
                'hostip': my_ip,
                'wait': "true"
            }

    response, content = http.request( proxyConnectURL,
                                          'POST',
                                          headers=proxyHeaders,
                                          body=dumps(proxyBody),
                                       )
    try:
        data = json.loads(content.decode())["connection"]["proxy"]

        data = data.split(":")
        print("[remot3]: IPfound: "+data[1][2:]+"::"+data[2])
        #return data[0]+":"+data[1],data[2]
        return data[1][2:],data[2]
    except KeyError:
        print("[remot3]: Key Error exception!")
        print("[remot3]: "+content)
        return "localhost", "5555"

def handle_getIp(req):
    developerKey="RUI2QzlDRjQtMTg0NC00RjU0LUE0NDItMDIwNDJGRDBGOERE"
    token = getToken(developerKey,req.username,req.password)
    device = getDevice(token,developerKey)
    serviceID = getService('Relay',device)
    if not serviceID == 'NULL':
        ip,port = proxyConnect(serviceID,token,developerKey)
        return getIpResponse(ip,port)
    return getIpResponse("localhost","5555")

if __name__ == '__main__':
    rospy.init_node('IpNode')
    s = rospy.Service('/Internet/getIp',getIp, handle_getIp)
    print(" ")
    print("[remot3]: IP fetcher running")

    heartbeat_pub = rospy.Publisher('/node_monitor/input/Heartbeat', heartbeat, queue_size = 10)
    heart_msg = heartbeat()
    heart_msg.header.frame_id = 'remot3'
    heart_msg.rate = 1

    while not rospy.is_shutdown():
        rospy.Rate(heart_msg.rate).sleep()
        heart_msg.header.stamp = rospy.Time.now()
        heartbeat_pub.publish(heart_msg)

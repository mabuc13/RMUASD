#!/usr/bin/env python3
import requests
import httplib2
import json
import rospy
from urllib.request import urlopen
from json import dumps

#"RUI2QzlDRjQtMTg0NC00RjU0LUE0NDItMDIwNDJGRDBGOERE"
def getToken(developerKey):
    url = "https://api.remot3.it/apv/v23.5/user/login"
    print ("Connecting to: "+ url)
    payload = "{ \"username\" : \"waarbubble@gmail.com\", \"password\" : \"RMUASDProject\" }"
    headers = {
        'developerkey': developerKey,
        'content-type': "application/json",
        'cache-control': "no-cache"
        }

    response = requests.request("POST", url, data=payload, headers=headers)
    jsonFile = response.json()
    print("Found token: "+ jsonFile['token'])
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

    print ("Connecting to: " + deviceListURL)
    response, content = http.request( deviceListURL,
                                          'GET',
                                          headers=deviceListHeaders)


    deviceJson = json.loads(content.decode())
    print("Device found: " + deviceJson['status'])
    #print(deviceJson['devices'])
    #print(content)
    return deviceJson['devices']

def getService(name,device):
    for service in device:
        if service['devicealias'] == name:
            print ("service found: "+ service['devicealias'])
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
    print("my I: " + my_ip)

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
        print(data)
    except KeyError:
        print("Key Error exception!")
        print(content)

if __name__ == '__main__':
    rospy.init_node('IpNode')

    developerKey="RUI2QzlDRjQtMTg0NC00RjU0LUE0NDItMDIwNDJGRDBGOERE"
    token = getToken(developerKey)
    device = getDevice(token,developerKey)
    serviceID = getService('Relay',device)
    if not serviceID == 'NULL':
       proxyConnect(serviceID,token,developerKey)

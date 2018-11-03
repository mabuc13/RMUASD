#!/usr/bin/env python3

import json
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list
from mavlink_defines import * # pylint: disable=W0614
import rospy

def import_plan(filename, target_sys=1, target_comp=0):
    with open(filename) as file:
        data = json.load(file)

    mission_list = mavlink_lora_mission_list()

    for itr, waypoint in enumerate(data['mission']['items']):
        
        curr = 0
        if itr == 0:
            curr = 1

        # convert all None-types to nan
        params = [float('nan') if param == None else param for param in waypoint['params'] ]
        command = waypoint['command']

        if command in SUPPORTED_GLOBAL_FRAMES:
            frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        elif command in SUPPORTED_MISSION_FRAMES:
            frame = MAV_FRAME_MISSION
        else:
            rospy.logwarn("Mission item ignored. Not supported on PX4 stack.")
            continue

        mission_item = mavlink_lora_mission_item_int(
            param1=params[0],
            param2=params[1],
            param3=params[2],
            param4=params[3],
            x=int(waypoint['params'][4]*1e7),
            y=int(waypoint['params'][5]*1e7),
            z=waypoint['params'][6],
            seq=waypoint['doJumpId'],
            command=command,
            current=curr,
            autocontinue=1,
            target_system=target_sys,
            target_component=target_comp,
            frame=frame
        )

        mission_list.waypoints.append(mission_item)
    
    return mission_list

if __name__ == "__main__":
    pass

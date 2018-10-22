#!/usr/bin/env python3

import json
from mavlink_lora.msg import mavlink_lora_mission_item_int, mavlink_lora_mission_list

MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6

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

        mission_item = mavlink_lora_mission_item_int(
            param1=params[0],
            param2=params[1],
            param3=params[2],
            param4=params[3],
            x=int(waypoint['params'][4]*1e7),
            y=int(waypoint['params'][5]*1e7),
            z=waypoint['params'][6],
            seq=waypoint['doJumpId'],
            command=waypoint['command'],
            current=curr,
            autocontinue=1,
            target_system=target_sys,
            target_component=target_comp,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        )

        mission_list.waypoints.append(mission_item)
    
    return mission_list

if __name__ == "__main__":
    with open("../plans/test_plan.plan") as file:
        data = json.load(file)

    mission_list = mavlink_lora_mission_list()

    target_sys = 1
    target_comp = 0

    for waypoint in data['mission']['items']:
        print("---------------------")

        params = [float('nan') if param == None else param for param in waypoint['params'] ]

        mission_item = mavlink_lora_mission_item_int(
            param1=params[0],
            param2=params[1],
            param3=params[2],
            param4=params[3],
            x=int(waypoint['params'][4]*1e7),
            y=int(waypoint['params'][5]*1e7),
            z=waypoint['params'][6],
            seq=waypoint['doJumpId'],
            command=waypoint['command'],
            current=0,
            autocontinue=1,
            target_system=target_sys,
            target_component=target_comp,
            frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
        )

        mission_list.waypoints.append(mission_item)

        

        # for key, value in waypoint.items():
        #     print("{}: {}".format(key,value))
        
    
    print("---------------------")

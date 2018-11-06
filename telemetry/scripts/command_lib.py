#!/usr/bin/env python3
#/***************************************************************************
# MavLink LoRa node (ROS) mission library
# Copyright (c) 2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
'''
Documentation: http://mavlink.org/messages/common

Revision
2018-06-13 KJ First published version
'''

# defines

import struct
from mavlink_lora.msg import mavlink_lora_msg
from mavlink_defines import * # pylint: disable=W0614

class command_lib():
    def __init__(self):
        self.msg = mavlink_lora_msg(sys_id=0, comp_id=0)   
        self.target_sys = 1
        self.target_comp = 1
        self.command = ""

    def set_target(self, target_sys, target_comp):
        # self.target_sys = target_sys
        # self.target_comp = target_comp    
        pass
    
    def pack_command_long(self, params, command, confirmation):
        self.msg.msg_id = MAVLINK_MSG_ID_COMMAND_LONG
        self.msg.payload_len = MAVLINK_MSG_ID_COMMAND_LONG_LEN
        self.msg.payload = struct.pack('<7fHBBB', params[0], params[1], params[2], params[3], params[4], params[5], params[6], command, self.target_sys, self.target_comp, confirmation)
        self.command = command
	# def unpack_mission_current(self, payload):
	# 	current = struct.unpack('<H', payload)
	# 	return current
		
	# def unpack_mission_item(self, payload):
	# 	(param1, param2, param3, param4, x, y, z, seq, cmd, _, _, _, cur, autocont) = struct.unpack('fffffffHHBBBBB', payload)
	# 	return [param1, param2, param3, param4, x, y, z, seq, cmd, cur, autocont]
	
if __name__ == "__main__":
	pass


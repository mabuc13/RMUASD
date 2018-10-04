#!/usr/bin/env python

import rospy
import struct
import codecs
from mavlink_lora.msg import mavlink_lora_msg
import serial
import time
from rtcm_msg import RTCM_msg

# parameters
mavlink_lora_sub_topic = '/mavlink_rx'
mavlink_lora_pub_topic = '/mavlink_tx'
update_interval = 0.2

MAVLINK_MSG_ID_GPS_RTCM_DATA = 233
MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN = 182
MAVLINK_MSG_ID_COMMAND_ACK = 77

# variables
msg = mavlink_lora_msg()
request_sent = False
first_msg_ok = False
sequence_id = 0

def on_mavlink_msg (msg):
	global first_msg_ok
	global target_sys
	if first_msg_ok == False:
		first_msg_ok = True
		target_sys = msg.sys_id

	if msg.msg_id == MAVLINK_MSG_ID_COMMAND_ACK:
		(command, result) = struct.unpack('<HB', msg.payload)	
		print command, result

def send_mavlink_rtcm(rtcm):
    # no need to set sys_id, comp_id or checksum, this is handled by the mavlink_lora node.
    msg.header.stamp = rospy.Time.now()
    msg.msg_id = MAVLINK_MSG_ID_GPS_RTCM_DATA
    msg.sys_id = 0
    msg.comp_id = 0

    global sequence_id

	# Payload
    sequence_id += 1
    sequence_id &= 0x1F
    fragment_id = 0
    fragmented = 0
    
    flags           = sequence_id << 3 
    flags           |= fragment_id << 1
    flags           |= fragmented

    data            = bytearray(180)
    data[0:len(rtcm)] = rtcm.get_bytearray()
    msg.payload_len = MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN
    msg.payload = struct.pack('<BB180B', flags, len(rtcm), data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31], data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39], data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47], data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55], data[56], data[57], data[58], data[59], data[60], data[61], data[62], data[63], data[64], data[65], data[66], data[67], data[68], data[69], data[70], data[71], data[72], data[73], data[74], data[75], data[76], data[77], data[78], data[79], data[80], data[81], data[82], data[83], data[84], data[85], data[86], data[87], data[88], data[89], data[90], data[91], data[92], data[93], data[94], data[95], data[96], data[97], data[98], data[99], data[100], data[101], data[102], data[103], data[104], data[105], data[106], data[107], data[108], data[109], data[110], data[111], data[112], data[113], data[114], data[115], data[116], data[117], data[118], data[119], data[120], data[121], data[122], data[123], data[124], data[125], data[126], data[127], data[128], data[129], data[130], data[131], data[132], data[133], data[134], data[135], data[136], data[137], data[138], data[139], data[140], data[141], data[142], data[143], data[144], data[145], data[146], data[147], data[148], data[149], data[150], data[151], data[152], data[153], data[154], data[155], data[156], data[157], data[158], data[159], data[160], data[161], data[162], data[163], data[164], data[165], data[166], data[167], data[168], data[169], data[170], data[171], data[172], data[173], data[174], data[175], data[176], data[177], data[178], data[179])
    mavlink_msg_pub.publish(msg)
    
    sequence_id += 1

if __name__ == "__main__":
	# launch node

	ser = serial.Serial(
		port='/dev/ttyUSB0',
		baudrate=38400,
		parity=serial.PARITY_NONE,
		stopbits=serial.STOPBITS_ONE,
		bytesize=serial.EIGHTBITS
	)

	if ser.isOpen():
		rospy.init_node('mavlink_lora_arm_disarm')
		mavlink_msg_pub = rospy.Publisher(mavlink_lora_pub_topic, mavlink_lora_msg, queue_size=0) # mavlink_msg publisher
		rospy.Subscriber(mavlink_lora_sub_topic, mavlink_lora_msg, on_mavlink_msg) # mavlink_msg subscriber
		rate = rospy.Rate(update_interval)
		rospy.sleep (1) # wait until everything is running

        byte_buffer = bytearray()

        while not (rospy.is_shutdown()):
            time.sleep(0.5)
            while ser.inWaiting() > 0:
                byte = ser.read(1)
                byte_buffer.extend(byte)

            while True:
                val = struct.pack(">B", 0xd3)
                idx = byte_buffer.find(val)
                print(len(byte_buffer))
                if idx != -1:
                    try:
                        if byte_buffer[idx+1] > 3:
                            del byte_buffer[0:idx+1]
                            break
                        # extract the header of the message (2nd and 3rd byte)
                        # idx+1 <= header < idx+3
                        header = byte_buffer[idx+1:idx+3]
                        # mask out first 6 bits
                        header[0] = header[0] & 0x02
                        hdr_len = 2
                        msg_len = int(codecs.encode(header, 'hex'), 16)
                        # msg_len = int.from_bytes(header, byteorder='big')
                        crc_len = 3

                        message = byte_buffer[idx:idx+hdr_len+msg_len+crc_len+1]
                        del byte_buffer[0:idx+hdr_len+msg_len+crc_len+1]

                        rtcm_obj = RTCM_msg(message)
                        rtcm_obj.write_to_file("RTCM.txt")
                        print(rtcm_obj)

                        send_mavlink_rtcm(rtcm_obj)


                    except IndexError:
                        # we don't have the entire message yet
                        break

                    # print(msg_len)
                    # print(hex(byte_buffer[idx]) + " " + hex(byte_buffer[idx+1]) + " " + hex(byte_buffer[idx+2]))
                else:
                    break
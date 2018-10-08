/***************************************************************************
# MavLink LoRa library
# MavLink long range communication library 
# Copyright (c) 2017-2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
# SDU UAS Center, http://sdu.dk/uas 
# All rights reserved.
#
# This library is based on the output from the MavLink generator
# http://www.mavlink.org which is released under the MIT license:
# https://opensource.org/licenses/MIT
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
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************
Revision
2017-11-27 KJ First released test version
2018-03-18 KJ Added unpacking of global_position_int and mission_current
2018-04-04 KJ Added printf debugs and unpacking of statustext
2018-04-10 KJ Significant rewrites in order to make it Arduino compatible
2018-06-11 KJ Added support for attitude
2018-09-19 KJ Rearranged unpack functions according to id
****************************************************************************/
/* includes */
#include <stdio.h>
#include <string.h>
#include "mavlink_lora_lib.h"
#include "checksum.h" /* https://github.com/mavlink/c_library_v1/blob/master/checksum.h */

/***************************************************************************/
/* defines */

const unsigned char DROP_MSGS[] = DROP_MSGS_WITH_ID; /* incoming messages to drop */
const unsigned char SLOW_MSGS[] = SLOW_DOWN_MSGS_WITH_ID; /* incoming messages to slow down */
unsigned long SLOW_TOUT[] = SLOW_DOWN_TIMEOUTS; /* incoming messages to drop */

/* unsigned char MAVLINK_MESSAGE_CRCS_V1[] = {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 241, 0, 115, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 150, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0}; */

/* unsigned char MAVLINK_MESSAGE_CRCS_V2[] = {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 163, 105, 151, 35, 150, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0}; */

/* ArduPilotMega version */
const unsigned char MAVLINK_MESSAGE_CRCS_V1[] = {50, 124, 137, 0, 237, 217, 104, 119, 0, 0, 0, 89, 0, 0, 0, 0, 0, 0, 0, 0, 214, 159, 220, 168, 24, 23, 170, 144, 67, 115, 39, 246, 185, 104, 237, 244, 222, 212, 9, 254, 230, 28, 28, 132, 221, 232, 11, 153, 41, 39, 78, 196, 0, 0, 15, 3, 0, 0, 0, 0, 0, 167, 183, 119, 191, 118, 148, 21, 0, 243, 124, 0, 0, 38, 20, 158, 152, 143, 0, 0, 0, 106, 49, 22, 143, 140, 5, 150, 0, 231, 183, 63, 54, 47, 0, 0, 0, 0, 0, 0, 175, 102, 158, 208, 56, 93, 138, 108, 32, 185, 84, 34, 174, 124, 237, 4, 76, 128, 56, 116, 134, 237, 203, 250, 87, 203, 220, 25, 226, 46, 29, 223, 85, 6, 229, 203, 1, 195, 109, 168, 181, 47, 72, 131, 127, 0, 103, 154, 178, 200, 134, 219, 208, 188, 84, 22, 19, 21, 134, 0, 78, 68, 189, 127, 154, 21, 21, 144, 1, 234, 73, 181, 22, 83, 167, 138, 234, 240, 47, 189, 52, 174, 229, 85, 159, 186, 72, 0, 0, 0, 0, 92, 36, 71, 98, 0, 0, 0, 0, 0, 134, 205, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 69, 101, 50, 202, 17, 162, 0, 0, 0, 0, 0, 0, 207, 0, 0, 0, 163, 105, 151, 35, 150, 0, 0, 0, 0, 0, 0, 90, 104, 85, 95, 130, 184, 81, 8, 204, 49, 170, 44, 83, 46, 0};


typedef enum MAV_PARAM_TYPE /* mavlink/common/common.h */
{
	MAV_PARAM_TYPE_UINT8=1, /* 8-bit unsigned integer | */
	MAV_PARAM_TYPE_INT8=2, /* 8-bit signed integer | */
	MAV_PARAM_TYPE_UINT16=3, /* 16-bit unsigned integer | */
	MAV_PARAM_TYPE_INT16=4, /* 16-bit signed integer | */
	MAV_PARAM_TYPE_UINT32=5, /* 32-bit unsigned integer | */
	MAV_PARAM_TYPE_INT32=6, /* 32-bit signed integer | */
	MAV_PARAM_TYPE_UINT64=7, /* 64-bit unsigned integer | */
	MAV_PARAM_TYPE_INT64=8, /* 64-bit signed integer | */
	MAV_PARAM_TYPE_REAL32=9, /* 32-bit floating-point | */
	MAV_PARAM_TYPE_REAL64=10, /* 64-bit floating-point | */
	MAV_PARAM_TYPE_ENUM_END=11 /*  | */
} MAV_PARAM_TYPE;

/***************************************************************************/
/* static variables */

short i;

char debug = 0;
unsigned char txbuf[TX_BUF_SIZE];
unsigned char rxbuf[RX_BUF_SIZE];
unsigned char tx_seq;
short rxbuf_cnt;
short txbuf_cnt;

short msg_begin, msg_next;
unsigned long msg_rx_cnt, msg_crc_err, msg_tx_cnt;
unsigned long msg_buf_overflow;
unsigned char drop_certain_msgs;
unsigned char slow_certain_msgs;

static unsigned char recorded_sysid;

unsigned char do_send_msg;
unsigned long slow_tout[12]; 
unsigned long param_mission_tout;

/***************************************************************************/
void ml_init(void)
{

	/* initialize variables */
	msg_begin = -1;
	msg_rx_cnt = 0;
	msg_crc_err = 0;
	msg_tx_cnt = 0;
	msg_buf_overflow = 0;
	tx_seq = 1;
	drop_certain_msgs = 1;
	slow_certain_msgs = 1;
	recorded_sysid = 0;
	param_mission_tout = 0;
}
/***************************************************************************/
unsigned long ml_messages_sent(void)
{
	return msg_tx_cnt;
}
/***************************************************************************/
unsigned long ml_messages_received(void)
{
	return msg_rx_cnt;
}
/***************************************************************************/
unsigned long ml_messages_crc_error(void)
{
	return msg_crc_err;
}
/***************************************************************************/
void ml_set_monitor_all(void)
{
	drop_certain_msgs = 0;
	slow_certain_msgs = 0;
}
/***************************************************************************/
mavlink_sys_status_t ml_unpack_msg_sys_status (unsigned char *payload)
{
	/* id=1 */
	/* mavlink/common/mavlink_msg_sys_status.h */
	mavlink_sys_status_t sys_status;

	sys_status.load = payload[12] | (payload[13] << 8);
	sys_status.voltage_battery = payload[14] | (payload[15] << 8);
	sys_status.current_battery = payload[16] | (payload[17] << 8);
	sys_status.errors_comm = payload[20] | (payload[21] << 8);

	return sys_status;
}
/***************************************************************************/
mavlink_gps_raw_int_t ml_unpack_msg_gps_raw_int (unsigned char *payload)
{
	/* id=24 */
	/* mavlink/common/mavlink_gps_raw_int.h */
	mavlink_gps_raw_int_t gri; 
	uint64_t *ui64p;
	int32_t *i32p;
	uint8_t *ui8p;
	uint16_t *ui16p;

	ui64p = (uint64_t *) (payload + 0);
	gri.time_usec = *ui64p;
	i32p = (int32_t *) (payload + 8);
	gri.lat = *i32p;
	i32p = (int32_t *) (payload + 12);
	gri.lon = *i32p;
	i32p = (int32_t *) (payload + 16);
	gri.alt = *i32p;
	ui16p = (uint16_t *) (payload + 20);
	gri.eph = *ui16p;
	ui16p = (uint16_t *) (payload + 22);
	gri.epv = *ui16p;
	ui16p = (uint16_t *) (payload + 24);
	gri.vel = *ui16p;
	ui16p = (uint16_t *) (payload + 26);
	gri.cog = *ui16p;
	ui8p = (uint8_t *) (payload + 28);
	gri.fix_type = *ui8p;
	ui8p = (uint8_t *) (payload + 29);
	gri.satellites_visible = *ui8p;

	return gri;
}
/***************************************************************************/
mavlink_attitude_t ml_unpack_msg_attitude (unsigned char *payload)
{
	/* id=30 */
	/* mavlink/common/mavlink_attitude.h */
	mavlink_attitude_t atti; 
	uint32_t *ui32p;
	float *fp;

	ui32p = (uint32_t *) (payload + 0);
	atti.time_boot_ms = *ui32p;
	fp = (float *) (payload + 4);
	atti.roll = *fp;
	fp = (float *) (payload + 8);
	atti.pitch = *fp;
	fp = (float *) (payload + 12);
	atti.yaw = *fp;

	return atti;
}
/***************************************************************************/
mavlink_global_position_int_t ml_unpack_msg_global_position_int (unsigned char *payload)
{
	/* id=33 */
	/* mavlink/common/mavlink_global_position_int.h */
	mavlink_global_position_int_t gpi; 
	int32_t *i32p;
	int16_t *i16p;
	uint16_t *ui16p;

	i32p = (int32_t *) (payload + 0);
	gpi.time_boot_ms = *i32p;
	i32p = (int32_t *) (payload + 4);
	gpi.lat = *i32p;
	i32p = (int32_t *) (payload + 8);
	gpi.lon = *i32p;
	i32p = (int32_t *) (payload + 12);
	gpi.alt = *i32p;
	i32p = (int32_t *) (payload + 16);
	gpi.relative_alt = *i32p;
	i16p = (int16_t *) (payload + 20);
	gpi.vx = *i16p;
	i16p = (int16_t *) (payload + 22);
	gpi.vx = *i16p;
	i16p = (int16_t *) (payload + 24);
	gpi.vx = *i16p;
	ui16p = (uint16_t *) (payload + 26);
	gpi.hdg = *ui16p;
	
	return gpi;
}
/***************************************************************************/
mavlink_mission_item_t ml_unpack_msg_mission_item (unsigned char *payload)
{
	/* id=39 */
	/* mavlink/common/mavlink_msg_mission_item.h */
	mavlink_mission_item_t item;
	float *fp;

	fp = (float *) (payload + 0);
	item.param1 = *fp;
	fp = (float *) (payload + 4);
	item.param2 = *fp;
	fp = (float *) (payload + 8);
	item.param3 = *fp;
	fp = (float *) (payload + 12);
	item.param4 = *fp;
	fp = (float *) (payload + 16);
	item.x = *fp;
	fp = (float *) (payload + 20);
	item.y = *fp;
	fp = (float *) (payload + 24);
	item.z = *fp;

	item.seq = payload[28] | (payload[29] << 8);
	item.command = payload[30] | (payload[31] << 8);
	item.target_system = payload[32];
	item.target_component = payload[33];
	item.frame = payload[34];
	item.current = payload[35];
	item.autocontinue = payload[36];

	return item;
}
/***************************************************************************/
mavlink_mission_item_int_t ml_unpack_msg_mission_item_int (unsigned char *payload)
{
	/* id=73 */
	/* mavlink/common/mavlink_msg_mission_item_int.h */
	mavlink_mission_item_int_t item;
	float *fp;
	uint32_t *ui32p;

	fp = (float *) (payload + 0);
	item.param1 = *fp;
	fp = (float *) (payload + 4);
	item.param2 = *fp;
	fp = (float *) (payload + 8);
	item.param3 = *fp;
	fp = (float *) (payload + 12);
	item.param4 = *fp;
	ui32p = (uint32_t *) (payload + 16);
	item.x = *ui32p;
	ui32p = (uint32_t *) (payload + 20);
	item.y = *ui32p;
	fp = (float *) (payload + 24);
	item.z = *fp;

	item.seq = payload[28] | (payload[29] << 8);
	item.command = payload[30] | (payload[31] << 8);
	item.target_system = payload[32];
	item.target_component = payload[33];
	item.frame = payload[34];
	item.current = payload[35];
	item.autocontinue = payload[36];

	return item;
}
/***************************************************************************/
mavlink_mission_request_int_t ml_unpack_msg_mission_request_int (unsigned char *payload)
{
	/* id=51 */
	/* mavlink/common/mavlink_msg_mission_request_int.h */
	mavlink_mission_request_int_t item;
	uint16_t *ui16p;

	ui16p = (uint16_t *) (payload + 0);
	item.seq = *ui16p;
	item.target_system = payload[2];
	item.target_component = payload[3];

	return item;
}
/***************************************************************************/
mavlink_mission_ack_t ml_unpack_msg_mission_ack (unsigned char *payload)
{
	/* id=47 */
	/* mavlink/common/mavlink_mission_ack */
	mavlink_mission_ack_t ack;

	ack.target_system = payload[0];
	ack.target_component = payload[1];
	ack.type = payload[2];

	return ack;
}
/***************************************************************************/
unsigned short ml_unpack_msg_mission_current (unsigned char *payload)
{
	/* id=42 */
	/* mavlink/common/mavlink_msg_mission_current.h */
	return payload[0] | (payload[1] << 8);
}
/***************************************************************************/
unsigned short ml_unpack_msg_mission_count (unsigned char *payload)
{
	/* id=44 */
	/* mavlink/common/mavlink_msg_mission_count.h */
	return payload[0] | (payload[1] << 8);
}
/***************************************************************************/
battery_status_t ml_unpack_msg_battery_status (unsigned char *payload)
{
	/* id=147 */
	/* OBS not yet completed */
	battery_status_t batt;

	return batt;
}
/***************************************************************************/
mavlink_statustext_t ml_unpack_msg_statustext (unsigned char *payload)
{
	/* id=253 */
	/* mavlink/common/mavlink_msg_statustext.h */
	mavlink_statustext_t statustext;
	
	statustext.severity = payload[0];
	memcpy (statustext.text, payload+1, 50);

	return statustext;
}
/***************************************************************************/
mavlink_command_ack_t ml_unpack_msg_command_ack (unsigned char *payload)
{
	/* id=77 */
	/* mavlink/common/mavlink_command_ack */
	mavlink_command_ack_t ack; 

	ack.command = payload[0] | (payload[1] << 8);
	ack.result = payload[2];
	
	return ack;
}
/***************************************************************************/
short ml_queue_msg (unsigned char *buf)
{
	unsigned char msg_id;
	unsigned char payload_len;
	unsigned short crc;
	unsigned char crc_extra;

	msg_tx_cnt++;

	/* encode the generic part of the header */
	buf[ML_POS_IDENT] = ML_NEW_PACKET_IDENT_V10;
	buf[ML_POS_PACKET_SEQ] = tx_seq++;
	if (tx_seq == 0)
		tx_seq = 1;

	/* add checksum */
	msg_id = buf[ML_POS_MSG_ID];
	payload_len = buf[ML_POS_PAYLOAD_LEN];
	crc = crc_calculate(buf+1, payload_len+5);
	crc_extra = MAVLINK_MESSAGE_CRCS_V1[msg_id];
	crc_accumulate(crc_extra, &crc);
	(buf+payload_len+ 6)[0] = (crc & 0xff);
	(buf+payload_len+ 7)[0] = (crc >> 8);

	/*printf ("%d\n", crc);
	int i;
	for (i=0; i<buf[ML_POS_PAYLOAD_LEN]+8; i++)
	{
			printf ("%03d ", buf[i]);
	}
	printf ("\n"); */

	/* update txbuf length */
	txbuf_cnt += buf[ML_POS_PAYLOAD_LEN] + 8;
}
/***************************************************************************/
void ml_queue_msg_generic (unsigned char sys_id, unsigned char comp_id, unsigned char msg_id, unsigned char payload_len, unsigned char *payload)
{
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);
	
	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = payload_len;
	buf[ML_POS_SYS_ID] = sys_id;
	buf[ML_POS_COMP_ID] = comp_id;
	buf[ML_POS_MSG_ID] = msg_id;

	/* param_index */
	for (i=0; i<payload_len; i++)
		buf[ML_POS_PAYLOAD + i] = payload[i];

	/* queue message */
	recorded_sysid = 0;
	ml_queue_msg(buf);
}

/***************************************************************************/
void ml_queue_msg_param_request_read (char *param_id)
{
	/* id=20 */
	/* reference: mavlink/common/mavlink_msg_param_request_read.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);
	
	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_PARAM_REQUEST_READ;

	/* param_index */
	buf[ML_POS_PAYLOAD + 0] = 0xff; /* param_index LSB, set param_index to -1 to use the param_id */
	buf[ML_POS_PAYLOAD + 1] = 0xff; /* param_index MSB */

	/* target_system */
	buf[ML_POS_PAYLOAD + 2] = MAV_SYS_ID_UA; 

	/* target_component */
	buf[ML_POS_PAYLOAD + 3] = 0;

	/* param_id */
	len = strlen(param_id);
	for (i=0; i<len; i++)
		buf[ML_POS_PAYLOAD + 4 + i] = param_id[i];
	for (; i<16; i++)
		buf[ML_POS_PAYLOAD + 4 + i] = 0;

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_param_request_list (void)
{
	/* id=21 */
	/* reference: mavlink/common/mavlink_msg_param_request_list.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);
	
	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_PARAM_REQUEST_LIST;

	/* target_system */
	buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; 

	/* target_component */
	buf[ML_POS_PAYLOAD + 1] = 0;

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_param_set(char *param_id, float param_value)
{
	/* id=23 */
	/* reference: mavlink/common/mavlink_msg_param_set.h */
	unsigned char i, len;
	unsigned char *pv;
	unsigned char *buf = (txbuf + txbuf_cnt);

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_PARAM_SET_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_PARAM_SET;

	/* param_value */
	pv = (unsigned char *) &param_value;
	buf[ML_POS_PAYLOAD + 0] = pv[0]; 
	buf[ML_POS_PAYLOAD + 1] = pv[1]; 
	buf[ML_POS_PAYLOAD + 2] = pv[2]; 
	buf[ML_POS_PAYLOAD + 3] = pv[3]; 
	
	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 4] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 5] = 0; /* target component */

	/* param_id */
	len = strlen(param_id);
	for (i=0; i<len; i++)
		buf[ML_POS_PAYLOAD + 6 + i] = param_id[i];
	for (; i<16; i++)
		buf[ML_POS_PAYLOAD + 6 + i] = 0;

	/* param_type */
	buf[ML_POS_PAYLOAD + 22] = 9; 

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_mission_request (unsigned short seq)
{
	/* id=40 */
	/* reference: mavlink/common/mavlink_msg_mission_request.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_REQUEST_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_REQUEST;

	/* seq */
	buf[ML_POS_PAYLOAD + 0] = seq & 0xff;
	buf[ML_POS_PAYLOAD + 1] = (seq>>8) & 0xff;
	
	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 2] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 3] = 0; /* target component */

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_mission_request_list(void)
{
	/* id=43 */
	/* reference: mavlink/common/mavlink_msg_mission_request_list.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_REQUEST_LIST;
	
	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 1] = 0; /* target component */

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_mission_item_int (float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, unsigned short seq, unsigned short command, unsigned char frame, unsigned char current, unsigned char autocontinue)
{
	/* id=73 */
	/* reference: mavlink/common/mavlink_msg_mission_item_int.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);
    unsigned char *pv;

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_ITEM_INT;

	/* parameters */
    pv = (unsigned char *) &param1;
	buf[ML_POS_PAYLOAD + 0] = pv[0];
	buf[ML_POS_PAYLOAD + 1] = pv[1];
	buf[ML_POS_PAYLOAD + 2] = pv[2];
	buf[ML_POS_PAYLOAD + 3] = pv[3];

    pv = (unsigned char *) &param2;
	buf[ML_POS_PAYLOAD + 4] = pv[0];
	buf[ML_POS_PAYLOAD + 5] = pv[1];
	buf[ML_POS_PAYLOAD + 6] = pv[2];
	buf[ML_POS_PAYLOAD + 7] = pv[3];

    pv = (unsigned char *) &param3;
	buf[ML_POS_PAYLOAD + 8] = pv[0];
	buf[ML_POS_PAYLOAD + 9] = pv[1];
	buf[ML_POS_PAYLOAD + 10] = pv[2];
	buf[ML_POS_PAYLOAD + 11] = pv[3];

    pv = (unsigned char *) &param4;
	buf[ML_POS_PAYLOAD + 12] = pv[0];
	buf[ML_POS_PAYLOAD + 13] = pv[1];
	buf[ML_POS_PAYLOAD + 14] = pv[2];
	buf[ML_POS_PAYLOAD + 15] = pv[3];

	/* coordinates */
    pv = (unsigned char *) &x;
	buf[ML_POS_PAYLOAD + 16] = pv[0];
	buf[ML_POS_PAYLOAD + 17] = pv[1];
	buf[ML_POS_PAYLOAD + 18] = pv[2];
	buf[ML_POS_PAYLOAD + 19] = pv[3];

    pv = (unsigned char *) &y;
	buf[ML_POS_PAYLOAD + 20] = pv[0];
	buf[ML_POS_PAYLOAD + 21] = pv[1];
	buf[ML_POS_PAYLOAD + 22] = pv[2];
	buf[ML_POS_PAYLOAD + 23] = pv[3];

    pv = (unsigned char *) &z;
	buf[ML_POS_PAYLOAD + 24] = pv[0];
	buf[ML_POS_PAYLOAD + 25] = pv[1];
	buf[ML_POS_PAYLOAD + 26] = pv[2];
	buf[ML_POS_PAYLOAD + 27] = pv[3];

	/* sequence */
	buf[ML_POS_PAYLOAD + 28] = seq & 0xff;
	buf[ML_POS_PAYLOAD + 29] = (seq>>8) & 0xff;

	/* command */
	buf[ML_POS_PAYLOAD + 30] = command & 0xff;
	buf[ML_POS_PAYLOAD + 31] = (command>>8) & 0xff;

	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 32] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 33] = 0; /* target component, 0 equals ALL */

	/* Frame */
	buf[ML_POS_PAYLOAD + 34] = frame;

	/* Current */
	buf[ML_POS_PAYLOAD + 35] = current;

	/* autocontinue */
	buf[ML_POS_PAYLOAD + 36] = autocontinue;

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_mission_count (unsigned short count)
{
	/* id=44 */
	/* reference: mavlink/common/mavlink_msg_mission_count.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_COUNT_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_COUNT;

	/* count (number of mission items to upload */
	buf[ML_POS_PAYLOAD + 0] = count & 0xff;
	buf[ML_POS_PAYLOAD + 1] = (count>>8) & 0xff;

	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 2] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 3] = 0; /* target component */

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_mission_clear_all(void)
{
    /* id=45 */
    /* reference: mavlink/common/mavlink_msg_mission_clear_all.h */
    unsigned char i, len;
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_CLEAR_ALL;

    /* system_id (target) */
    buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; /* UA is the target system */

    /* component_id (target) */
    buf[ML_POS_PAYLOAD + 1] = 0; /* target component */

    /* queue message */
    ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_mission_ack(void)
{
	/* id=47 */
	/* reference: mavlink/common/mavlink_msg_mission_ack.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_MISSION_ACK_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_MISSION_ACK;
	
	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 0] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 1] = 0; /* target component */

	/* type */
	buf[ML_POS_PAYLOAD + 2] = 0; /* type (MAVLINK_MISSION_RESULT enum) */

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_request_data_streem(void)
{
	/* id=66 */
	/* reference: mavlink/common/mavlink_msg_request_data_stream.h */
	unsigned char i, len;
	unsigned char *buf = (txbuf + txbuf_cnt);
	uint16_t *ui16p;

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_REQUEST_DATA_STREAM;

	/* req_message_rate */	
	ui16p = (uint16_t *) (buf + ML_POS_PAYLOAD + 2);
	
	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 2] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 3] = 0; /* target component */

	/* req_stream_id */
	buf[ML_POS_PAYLOAD + 4] = 0; /* req_stream_id (uint8_t) */
	buf[ML_POS_PAYLOAD + 5] = 0; /* start_stop (uint8_t) */

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_command_long(unsigned short cmd_id, float param1, float param2, float param3, float param4, float param5, float param6, float param7, unsigned int confirmation )
{
	/* id=76 */
	/* reference: mavlink/common/mavlink_msg_command_long */
	unsigned char *buf = (txbuf + txbuf_cnt);
    unsigned char *pv;

	/* encode part of the header */
	buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_COMMAND_LONG_LEN;
	buf[ML_POS_SYS_ID] = recorded_sysid;
	buf[ML_POS_COMP_ID] = 0;
	buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_COMMAND_LONG;

	/* param_value */
    pv = (unsigned char *) &param1;
	buf[ML_POS_PAYLOAD + 0]  = pv[0];
	buf[ML_POS_PAYLOAD + 1]  = pv[1];
	buf[ML_POS_PAYLOAD + 2]  = pv[2];
	buf[ML_POS_PAYLOAD + 3]  = pv[3];

    pv = (unsigned char *) &param2;
	buf[ML_POS_PAYLOAD + 4]  = pv[0];
	buf[ML_POS_PAYLOAD + 5]  = pv[1];
	buf[ML_POS_PAYLOAD + 6]  = pv[2];
	buf[ML_POS_PAYLOAD + 7]  = pv[3];

    pv = (unsigned char *) &param3;
	buf[ML_POS_PAYLOAD + 8]  = pv[0];
	buf[ML_POS_PAYLOAD + 9]  = pv[1];
	buf[ML_POS_PAYLOAD + 10]  = pv[2];
	buf[ML_POS_PAYLOAD + 11]  = pv[3];

    pv = (unsigned char *) &param4;
	buf[ML_POS_PAYLOAD + 12] = pv[0];
	buf[ML_POS_PAYLOAD + 13] = pv[1];
	buf[ML_POS_PAYLOAD + 14] = pv[2];
	buf[ML_POS_PAYLOAD + 15] = pv[3];

    pv = (unsigned char *) &param5;
	buf[ML_POS_PAYLOAD + 16] = pv[0];
	buf[ML_POS_PAYLOAD + 17] = pv[1];
	buf[ML_POS_PAYLOAD + 18] = pv[2];
	buf[ML_POS_PAYLOAD + 19] = pv[3];

    pv = (unsigned char *) &param6;
	buf[ML_POS_PAYLOAD + 20] = pv[0];
	buf[ML_POS_PAYLOAD + 21] = pv[1];
	buf[ML_POS_PAYLOAD + 22] = pv[2];
	buf[ML_POS_PAYLOAD + 23] = pv[3];

    pv = (unsigned char *) &param7;
	buf[ML_POS_PAYLOAD + 24] = pv[0];
	buf[ML_POS_PAYLOAD + 25] = pv[1];
	buf[ML_POS_PAYLOAD + 26] = pv[2];
	buf[ML_POS_PAYLOAD + 27] = pv[3];

	/* command_id */
	buf[ML_POS_PAYLOAD + 28] = cmd_id & 0xff;
	buf[ML_POS_PAYLOAD + 29] = (cmd_id>>8) & 0xff;
	
	/* system_id (target) */
	buf[ML_POS_PAYLOAD + 30] = MAV_SYS_ID_UA; /* UA is the target system */

	/* component_id (target) */
	buf[ML_POS_PAYLOAD + 31] = 0; /* target component */
	
	/* confirmation */
	buf[ML_POS_PAYLOAD + 32] = confirmation;

	/* queue message */
	ml_queue_msg(buf);
}
/***************************************************************************/
void ml_queue_msg_heartbeat(void)
{
    /* id=0 */
    /* reference: mavlink/common/mavlink_msg_heartbeat */
    unsigned char *buf = (txbuf + txbuf_cnt);

    /* encode part of the header */
    buf[ML_POS_PAYLOAD_LEN] = MAVLINK_MSG_ID_HEARTBEAT_LEN;
    buf[ML_POS_SYS_ID] = recorded_sysid;
    buf[ML_POS_COMP_ID] = 0;
    buf[ML_POS_MSG_ID] = MAVLINK_MSG_ID_HEARTBEAT;

    /* pack message with no data, as no data is needed, but need to pack properly for correct msg format */

    /* custom mode */
    buf[ML_POS_PAYLOAD + 0] = 0;

	/* type. 6 = MAV_TYPE_GCS */
	buf[ML_POS_PAYLOAD + 4] = 6;

	/* autopilot type 8 = Invalid */
	buf[ML_POS_PAYLOAD + 5] = 8;

	/* base_mode type 1 = custom mode enabled */
	buf[ML_POS_PAYLOAD + 6] = 1;

	/* system_status type 4 = active */
	buf[ML_POS_PAYLOAD + 7] = 4;

	/* queue message */
    ml_queue_msg(buf);
}
/***************************************************************************/
short ml_rx_update(unsigned long now, unsigned char *rxbuf_new, short rxbuf_new_cnt)
{
	char result = 0;
	short i, j, count;
	unsigned char c;

	/* check for buffer owerflow */
	if (rxbuf_cnt + rxbuf_new_cnt > RX_BUF_SIZE)
	{
		rxbuf_cnt = 0;
		result = -1;
		msg_buf_overflow++;
		if (debug)
			printf ("Buffer overflow\n");
	}
	else
	{
		short seek_from = rxbuf_cnt;
		char maybe_more = 1;
		txbuf_cnt = 0;

		/* add new bytes to buffer */
  	/*for (i=0; i<rxbuf_new_cnt; i++)
  		rxbuf[rxbuf_cnt++] = rxbuf_new[i];*/
  	memcpy (rxbuf+rxbuf_cnt , rxbuf_new, rxbuf_new_cnt);
  	rxbuf_cnt += rxbuf_new_cnt;
	  	
		while (maybe_more == 1)
		{
			maybe_more = 0;
			if (msg_begin < 0) /* try to find a packet start */
			{
				for (i=seek_from; i<rxbuf_cnt; i++)
				{
					if (rxbuf[i] == ML_NEW_PACKET_IDENT_V10 && msg_begin < 0)
						msg_begin = i;
				}
			}

			/* if we have found a packet start and the packet len > minimum */
			if (msg_begin >= 0 && rxbuf_cnt >= msg_begin + 8) 
			{
				short payload_len = rxbuf[msg_begin + ML_POS_PAYLOAD_LEN];
				short msg_next = msg_begin + payload_len + 8; /* actually beginning of next */
		
				/* if we have a complete packet */
				if (rxbuf_cnt >= msg_next)
				{			
					unsigned char crc_ok;					
					unsigned char msg_id = rxbuf[msg_begin + ML_POS_MSG_ID];
					
					/* if the checksum is valid */
					unsigned char crc_lsb = rxbuf[msg_begin + payload_len + 6];
					unsigned char crc_msb = rxbuf[msg_begin + payload_len + 7];
					unsigned short crc = crc_calculate(rxbuf+msg_begin+1, payload_len+5);
					unsigned char crc_extra = MAVLINK_MESSAGE_CRCS_V1[msg_id];
					crc_accumulate(crc_extra, &crc);
					crc_ok = ((crc & 0xff) == crc_lsb && (crc >> 8) == crc_msb);

					if (crc_ok)
					{
						msg_rx_cnt++;
						do_send_msg = 1;

						/* if first time record the sys_id */	
						if (msg_id == 0 && recorded_sysid == 0)
							recorded_sysid = rxbuf[msg_begin + ML_POS_SYS_ID];

						/* check if included in the drop_msgs list */
						if (drop_certain_msgs != 0) 
						{
							for (i=0; i<sizeof(DROP_MSGS); i++)	
							{
								if (msg_id == DROP_MSGS[i])
									do_send_msg = 0;
							}
						}
						
						/* check if param or mission sequence is ongoing */
						if (do_send_msg == 1)
						{
							if (msg_id==20 || msg_id==21 || msg_id==22 || msg_id==23) /* param msgs */
							{
								param_mission_tout = now + PARAM_TOUT;
							}
							else if (msg_id==37 || msg_id==39 || msg_id==40 || msg_id==43 || msg_id==44 || msg_id==47) /* mission item transactions */
							{
								param_mission_tout = now + MISSION_TOUT;
							}
							else if (msg_id != 0)  /* heartbeat must get through */
							{
					   		if (now < param_mission_tout)
						 		 	do_send_msg = 0;
						  }
						}

						/* check if we should slow down this particular message id */
						if (do_send_msg == 1 && slow_certain_msgs != 0)
						{
							for (i=0; i<sizeof(SLOW_MSGS); i++)	
							{
								if (msg_id == SLOW_MSGS[i])
								{
									if (now >= slow_tout[i])
									{
						    		slow_tout[i] = now + SLOW_TOUT[i];
						    	}
						    	else
						    	{
						    		do_send_msg = 0;
						    	}
						  	}
						  }
						}	

					  /* handle packet */
					  if (do_send_msg == 1)
						{
							result ++;						

							ml_parse_msg (rxbuf + msg_begin);

							/*printf ("%ld accepted %d\n", ms, msg_id);*/
						}
							/*else printf ("%ld dropped %d\n", ms, msg_id);  */
					}
					else
					{
						msg_crc_err++;
						if (debug && msg_crc_err != 1) /* dischard first CRC error which usually occurs at startup */
						{
							printf ("CRC error (len %d): ", (msg_next - msg_begin));
							for (i=msg_begin; i<msg_next; i++)
							{
								printf ("%03d ", rxbuf[i]);
							}
							printf ("\n");

						}
					}
 
					/* remove packet from rxbuf */
					for (i=msg_next, j=0; i<rxbuf_cnt; i++, j++)
					{
						rxbuf[j] = rxbuf[i];
					}
					rxbuf_cnt -= msg_next;
					/* printf ("rxbuf_cnt_after %d\n", rxbuf_cnt); */
					
					msg_begin = -1;
					seek_from = 0;				
					maybe_more = 1;
					/* printf ("repeat\n");  */
				}
			}
	  }
	}
	return result;
}
/***************************************************************************/


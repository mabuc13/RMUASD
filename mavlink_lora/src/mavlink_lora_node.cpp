/***************************************************************************
# MavLink LoRa node (ROS)
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
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************

Revision
2018-04-17 KJ First released test version
2018-05-29 KJ Added support for GPS_RAW_INT messages, corrected voltage handling
2018-06-12 KJ Added attitude topic (works with AutoQuad for now), added
              serial lib making it ROS Melodic compatible, added parameters
              for serial dev. and baud.
****************************************************************************/
/* includes */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mavlink_lora/mavlink_lora_msg.h>	
#include <mavlink_lora/mavlink_lora_pos.h>	
#include <mavlink_lora/mavlink_lora_attitude.h>	
#include <mavlink_lora/mavlink_lora_status.h>	

extern "C"
{
	#include "serial.h"
	#include "mavlink_lora_lib.h"
}
/***************************************************************************/
/* defines */

#define RX_BUFFER_SIZE 16000
/***************************************************************************/
/* global variables */

int ser_ref;
struct termios oldtio;
ros::Time last_heard;
ros::Time sys_status_last_heard;
ros::Time status_msg_sent;
uint16_t sys_status_voltage_battery;
unsigned long secs_init;
ros::Publisher msg_pub, pos_pub, atti_pub, status_pub;
uint8_t rx_buffer[RX_BUFFER_SIZE];
unsigned short msg_id_global_position_int_received;

/***************************************************************************/
void mavlink_tx_callback(const mavlink_lora::mavlink_lora_msg::ConstPtr& msg)
{
	unsigned char *payload =	(unsigned char *) &msg->payload.front();
	ml_queue_msg_generic(msg->sys_id, msg->comp_id, msg->msg_id, msg->payload_len, payload);
	ml_tx_update();
}
/***************************************************************************/
void ml_send_status_msg(void)
{
	mavlink_lora::mavlink_lora_status status;
	status.header.stamp = ros::Time::now();
	status.last_heard = last_heard;
	status.last_heard_sys_status = sys_status_last_heard;
	status.batt_volt = sys_status_voltage_battery;
	status.msg_sent_gcs = ml_messages_sent();
	status.msg_received_gcs = ml_messages_received();
	status.msg_dropped_gcs = ml_messages_crc_error();
	status_pub.publish(status);
	status_msg_sent = ros::Time::now();
}
/***************************************************************************/
void ml_parse_msg(unsigned char *msg)
{
	last_heard = ros::Time::now();

	// extract message info
	mavlink_lora::mavlink_lora_msg m;
	m.header.stamp = last_heard;
	m.payload_len = msg[ML_POS_PAYLOAD_LEN];
	m.seq = msg[ML_POS_PACKET_SEQ];
	m.sys_id = msg[ML_POS_SYS_ID];
	m.comp_id = msg[ML_POS_COMP_ID];
	m.msg_id = msg[ML_POS_MSG_ID];
	
	int i;
	for (i=0; i<m.payload_len; i++)
		m.payload.push_back(msg[ML_POS_PAYLOAD + i]);
	
	unsigned char crc_lsb = msg[6 + m.payload_len];
	unsigned char crc_msb = msg[7 + m.payload_len];
	m.checksum = (8 << crc_msb) | crc_lsb;
	
	// publish the message
	msg_pub.publish(m);

	// handle pos messages
	if (m.msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
	{
		mavlink_lora::mavlink_lora_pos pos;
		pos.header.stamp = last_heard;
		mavlink_global_position_int_t glob_pos = ml_unpack_msg_global_position_int (&m.payload.front());
		pos.time_usec = (uint64_t) glob_pos.time_boot_ms*1000;
		pos.lat = glob_pos.lat / 1e7;
		pos.lon = glob_pos.lon / 1e7;
		pos.alt = glob_pos.alt / 1e3;
		pos.relative_alt = glob_pos.relative_alt / 1e3;
		pos.heading = glob_pos.hdg /100;
		pos_pub.publish(pos);
		msg_id_global_position_int_received = true;
	}

	if (m.msg_id == MAVLINK_MSG_ID_GPS_RAW_INT && msg_id_global_position_int_received == false)
	{
		mavlink_lora::mavlink_lora_pos pos;
		pos.header.stamp = last_heard;
		mavlink_gps_raw_int_t gri = ml_unpack_msg_gps_raw_int (&m.payload.front());
		pos.time_usec = gri.time_usec;
		pos.lat = gri.lat / 1e7;
		pos.lon = gri.lon / 1e7;
		pos.alt = gri.alt / 1e3;
		pos.relative_alt = -1;
		pos.heading = -1;
		pos_pub.publish(pos);
	}
	
	// handle attitude messages
	if (m.msg_id == MAVLINK_MSG_ID_ATTITUDE)
	{
		mavlink_lora::mavlink_lora_attitude atti;
		atti.header.stamp = last_heard;
		mavlink_attitude_t a = ml_unpack_msg_attitude (&m.payload.front());
		atti.time_usec = (uint64_t) a.time_boot_ms*1000;
		atti.yaw = a.yaw;
		atti.pitch = a.pitch;
		atti.roll = a.roll;
		atti_pub.publish(atti);
	}
	
	// handle state messages
	if	(m.msg_id == MAVLINK_MSG_ID_SYS_STATUS)
	{
	sys_status_last_heard = last_heard; 
		mavlink_sys_status_t sys_status = ml_unpack_msg_sys_status (&m.payload.front());
	sys_status_voltage_battery = sys_status.voltage_battery;
	ml_send_status_msg();
	}
}
/***************************************************************************/
void ml_tx_update (void)
{
	int bytes_written = ser_send(ser_ref, txbuf, txbuf_cnt);
	txbuf_cnt = 0; 
}
/***************************************************************************/
unsigned long millis(void)
{
	struct timeval te; 
	gettimeofday(&te, NULL); /* get current time */

	if (secs_init == 0)
	{
		secs_init = te.tv_sec;
	}

	return ((unsigned long) (te.tv_sec - secs_init)*1000 + te.tv_usec/1000);
}
/***************************************************************************/
int main (int argc, char** argv)
{
	ros::init(argc, argv, "mavlink_lora_node");
	ros::NodeHandle n;
	ros::NodeHandle nh("~"); // private parameters

	/* read from parameter server */
	std::string serial_device;
	int serial_baudrate;
	nh.param<std::string> ("serial_device", serial_device, "/dev/ttyACM0");
	nh.param ("serial_baudrate", serial_baudrate, 115200);

	/* initialize variables */
	ros::Time begin = ros::Time::now();
	msg_id_global_position_int_received = false;

	ros::Subscriber write_sub = n.subscribe("mavlink_tx", 10, mavlink_tx_callback);
	msg_pub = n.advertise<mavlink_lora::mavlink_lora_msg>("mavlink_rx", 1);
	pos_pub = n.advertise<mavlink_lora::mavlink_lora_pos>("mavlink_pos", 1);
	atti_pub = n.advertise<mavlink_lora::mavlink_lora_attitude>("mavlink_attitude", 1);
	status_pub = n.advertise<mavlink_lora::mavlink_lora_status>("mavlink_status", 1);

	/* initialize serial port */
	ROS_INFO("Opening serial device: %s baudrate %d", serial_device.c_str(), serial_baudrate);
	if (ser_open(&ser_ref, &oldtio, (char *) serial_device.c_str(), serial_baudrate))
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	ROS_INFO_STREAM("Serial Port initialized");
	
	/* initialize mavlink lora library */
	ml_init();
	ml_set_monitor_all();

	/* ros main loop */
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		ros::spinOnce();

		/* check if it is time to send a status message */
		if (status_msg_sent + ros::Duration(1) <= ros::Time::now())
		ml_send_status_msg();

		int cnt = ser_receive (ser_ref, rx_buffer, RX_BUFFER_SIZE);
		if(cnt > 0)
		{
				ml_rx_update(millis(), rx_buffer, cnt);
		}
		loop_rate.sleep();
	}
	
	/* closing down */
	ser_close (ser_ref, oldtio);
}
/***************************************************************************/


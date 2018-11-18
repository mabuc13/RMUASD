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
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <mavlink_lora/mavlink_lora_msg.h>	
#include <mavlink_lora/mavlink_lora_pos.h>	
#include <mavlink_lora/mavlink_lora_attitude.h>	
#include <mavlink_lora/mavlink_lora_status.h>
#include <mavlink_lora/mavlink_lora_mission_item_int.h>
#include <mavlink_lora/mavlink_lora_mission_list.h>
#include <mavlink_lora/mavlink_lora_mission_partial_list.h>
#include <mavlink_lora/mavlink_lora_mission_ack.h>
#include <mavlink_lora/mavlink_lora_command_ack.h>
#include <mavlink_lora/mavlink_lora_command_start_mission.h>
#include <mavlink_lora/mavlink_lora_command_set_mode.h>
#include <mavlink_lora/mavlink_lora_new_local_pos_sp.h>
#include <mavlink_lora/mavlink_lora_new_local_vel_sp.h>
#include <mavlink_lora/mavlink_lora_new_local_acc_sp.h>


extern "C"
{
	#include "serial.h"
	#include "mavlink_lora_lib.h"
}
/***************************************************************************/
/* defines */

#define RX_BUFFER_SIZE 16000
#define DEFAULT_TIMEOUT_TIME 1.5
#define DEFAULT_ACK_TIMEOUT 10
#define MISSION_ITEM_TIMEOUT_TIME 1.5
#define MISSION_MAX_RETRIES 5
#define HEARTBEAT_RATE 1
/***************************************************************************/
/* global variables */

int ser_ref; 
struct termios oldtio;
ros::Time last_heard;
ros::Time sys_status_last_heard;
ros::Time status_msg_sent;
ros::Time heartbeat_msg_sent;
uint16_t sys_status_voltage_battery;
unsigned long secs_init;
ros::Publisher msg_pub, pos_pub, atti_pub, status_pub, mission_ack_pub, command_ack_pub;
uint8_t rx_buffer[RX_BUFFER_SIZE];
unsigned short msg_id_global_position_int_received;
uint8_t recorded_sysid;

/* Mission upload operations variables */
unsigned short mission_up_count = 0; /*< Total count of mission elements to be uploaded*/
int mission_up_index = -1; /*< Current mission item getting uploaded */
uint16_t start_index = 0;
uint16_t end_index = 0;
std::vector<mavlink_lora::mavlink_lora_mission_item_int> missionlist; /*< list of all waypoints for upload */
bool mission_uploading = false; /*< Are we uploading a mission atm. Needed to know what messages/timeouts to react to*/
ros::Timer mission_up_timeout;// = ros::NodeHandle::createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_up_timeout_callback, true, false); /*< Timer for timeouts when doing mission transmissions */
ros::Timer mission_ack_timeout;
int mission_retries = 0;

int items_sent = 0;

/* Offboard related variables */
uint16_t local_sp_typemask = 0x03F8;
uint8_t local_sp_frame = 1;
float local_sp_x = 0;
float local_sp_y = 0;
float local_sp_z = -5;
float local_sp_vx = 0;
float local_sp_vy = 0;
float local_sp_vz = 0;
float local_sp_ax = 0;
float local_sp_ay = 0;
float local_sp_az = 0;
float local_sp_yaw = 0;
float local_sp_yaw_rate = 0;

/* Command Protocol */
unsigned int confirmation = 0; // increments for each timeout of same command. Useful to monitor if a command should be killed
ros::Timer command_timeout; // timeout for commands.
mavlink_command_long_t last_cmd_long; // Saving parameters for resending last command

/***************************************************************************/
/* Function Headers, for reference purposes */
void mission_up_count_timeout_callback(const ros::TimerEvent&);
void mission_up_item_timeout_callback(const ros::TimerEvent&);
void mission_up_partial_timeout_callback(const ros::TimerEvent&);
void mission_clear_all_timeout_callback(const ros::TimerEvent&);
void mission_ack_timeout_callback(const ros::TimerEvent&);
void command_long_timeout_callback(const ros::TimerEvent&);
void ml_send_mission_clear_all();
void ml_send_mission_count();
void ml_send_partial_list();
void ml_send_mission_item_int();
std::string mission_result_parser(uint8_t result);
std::string command_result_parser(uint8_t result);

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
    status.system_id = ml_recorded_sys_id();
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

    recorded_sysid = m.sys_id;

	// handle pos messages
	if (m.msg_id == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
	{
		mavlink_lora::mavlink_lora_pos pos;
		pos.header.stamp = last_heard;
		mavlink_global_position_int_t glob_pos = ml_unpack_msg_global_position_int (&m.payload.front());
		pos.time_usec = (uint64_t) glob_pos.time_boot_ms*1000;
        pos.system_id = ml_recorded_sys_id();
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
        pos.system_id = ml_recorded_sys_id();
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
        atti.system_id = ml_recorded_sys_id();
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

	//Handle Mission upload messages
	/* Forcing it to use INT variants for best possible precision. */
	if (m.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST_INT || m.msg_id == MAVLINK_MSG_ID_MISSION_REQUEST)
    {
	    // stop mission ack timout
        mission_ack_timeout.stop();
        ros::NodeHandle nh;
        mission_ack_timeout = nh.createTimer(ros::Duration(DEFAULT_ACK_TIMEOUT), mission_ack_timeout_callback, true, true);
        
        //unpack and update requested seq
	    mavlink_mission_request_int_t request = ml_unpack_msg_mission_request_int(&m.payload.front());

        // only answer a request for a sequence number the first time
        if(mission_up_index == request.seq)
            ROS_INFO_STREAM("Already sent that sequence number once");
        else
        {
            //stop timer
            mission_up_timeout.stop();

            //reset retries
            mission_retries = 0;

            mission_up_index = request.seq;

            //send next item
            ml_send_mission_item_int();

            ROS_INFO_STREAM("Next item asked for:" + std::to_string(mission_up_index)); //TODO agree on what to print, and if debug prints can be enabled from launchfile or so?
        }
            
    }

    //handle mission ack
    if (m.msg_id == MAVLINK_MSG_ID_MISSION_ACK)
    {
        //stop timers
        mission_up_timeout.stop();
        mission_ack_timeout.stop();

        //reset retries
        mission_retries = 0;

        //unpack
        mavlink_mission_ack_t ack = ml_unpack_msg_mission_ack(&m.payload.front());

        mission_uploading = false;

        //respond back with result
        mavlink_lora::mavlink_lora_mission_ack msg;
        msg.drone_id = m.sys_id;
        msg.result = ack.type;
        msg.result_text = mission_result_parser(ack.type);

        mission_ack_pub.publish(msg);

        // DEBUG
        ROS_INFO_STREAM(msg.result_text);
    }

    //handle command ack
    if (m.msg_id == MAVLINK_MSG_ID_COMMAND_ACK)
    {
        //stop timer
        command_timeout.stop();

        //reset confirmation
        confirmation = 0;

        //unpack
        mavlink_command_ack_t ack = ml_unpack_msg_command_ack(&m.payload.front());

        //respond back with result
        mavlink_lora::mavlink_lora_command_ack ack_msg;
        ack_msg.drone_id = recorded_sysid;
        ack_msg.command = ack.command;
        ack_msg.result = ack.result;
        ack_msg.result_text = command_result_parser(ack.result);

        //publish
        command_ack_pub.publish(ack_msg);

        //DEBUG
        ROS_INFO_STREAM(ack_msg.result_text);
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
/****************************** COMMAND_LONG *******************************/
void ml_send_command_long(unsigned short cmd_id, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    ROS_INFO_STREAM("Sending Command_long with id: " + std::to_string(cmd_id) + ", and confirmation: " + std::to_string(confirmation));

    //save command as last command
    last_cmd_long.command = cmd_id;
    last_cmd_long.param1 = p1;
    last_cmd_long.param2 = p2;
    last_cmd_long.param3 = p3;
    last_cmd_long.param4 = p4;
    last_cmd_long.param5 = p5;
    last_cmd_long.param6 = p6;
    last_cmd_long.param7 = p7;

    //queue msg
    ml_queue_msg_command_long(cmd_id, p1, p2, p3, p4, p5, p6, p7, confirmation);

    //update tx
    ml_tx_update();

    //start timeout
    ros::NodeHandle nh;
    command_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), command_long_timeout_callback, true, true);
}
/***************************************************************************/
void command_long_timeout_callback(const ros::TimerEvent&)
{
    //if timeout triggers, increment confirmation and resend last command
    confirmation++;

    ml_send_command_long(last_cmd_long.command, last_cmd_long.param1, last_cmd_long.param2, last_cmd_long.param3, last_cmd_long.param4, last_cmd_long.param5, last_cmd_long.param6, last_cmd_long.param7 );
}
/***************************************************************************/
std::string command_result_parser(uint8_t result)
{
    switch(result)
    {
        case 0:
            return "MAV_RESULT_ACCEPTED";
        case 1:
            return "MAV_RESULT_TEMPORARILY_REJECTED";
        case 2:
            return "MAV_RESULT_DENIED";
        case 3:
            return "MAV_RESULT_UNSUPPORTED";
        case 4:
            return "MAV_RESULT_FAILED";
        case 5:
            return "MAV_RESULT_IN_PROGRESS";
        default:
            return "DIDN'T RECOGNIZE RESULT CODE";
    }
}
/******************************* MISSIONS **********************************/
void ml_new_mission_callback(const mavlink_lora::mavlink_lora_mission_list::ConstPtr& msg)
{
    //Received new mission on topic, start uploading
    mission_up_count = msg->waypoints.size();
    mission_up_index = -1;
    missionlist = msg->waypoints;

    ROS_INFO_STREAM("New mission. Length:" + std::to_string(mission_up_count));

    //Send mission count
    ml_send_mission_count();

    //Set status to uploading mission. Currently not used, but maybe use it to make sure you can't upload a new mission while another mission gets uploaded?
    mission_uploading = true;

}
void ml_partial_mission_callback(const mavlink_lora::mavlink_lora_mission_partial_list::ConstPtr& msg)
{
    //Received new mission on topic, start uploading
    mission_up_count = msg->waypoints.size();
    mission_up_index = msg->start_index;


    if(msg->end_index >= missionlist.size())
    {
        ROS_WARN_STREAM("Index for partial mission is out of bounds. Aborting");
        return;
    }

    int itr = msg->start_index;
    for(auto waypoint : msg->waypoints)
    {
        missionlist[itr++] = waypoint;
    }

    ROS_INFO_STREAM("New partial mission. Length:" + std::to_string(mission_up_count));

    //Send mission count
    ml_send_partial_list();

    //Set status to uploading mission. Currently not used, but maybe use it to make sure you can't upload a new mission while another mission gets uploaded?
    mission_uploading = true;

}
void ml_mission_clear_all_callback(const std_msgs::Empty::ConstPtr& msg)
{
    //queue command for clearing all missions
    ml_send_mission_clear_all();

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_clear_all_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_mission_count()
{
    ROS_INFO_STREAM("Sending mission count");
    // queue msg
    ml_queue_msg_mission_count(mission_up_count);

    // update TX
    ml_tx_update();

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_up_count_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_partial_list()
{
    ROS_INFO_STREAM("Sending partial list");
    // queue msg
    ml_queue_msg_write_partial_list(start_index, end_index);

    // update TX
    ml_tx_update();

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_up_partial_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_mission_clear_all()
{
    ROS_INFO_STREAM("Sending mission clear all");
    // queue msg
    ml_queue_msg_mission_clear_all();

    // update TX
    ml_tx_update();

    // start timer
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(DEFAULT_TIMEOUT_TIME), mission_clear_all_timeout_callback, true, true);
}
/***************************************************************************/
void ml_send_mission_item_int()
{   
    //send mission item.
    mavlink_lora::mavlink_lora_mission_item_int item = missionlist[mission_up_index];
    ml_queue_msg_mission_item_int(item.param1, item.param2, item.param3, item.param4, item.x, item.y, item.z, mission_up_index, item.command, item.frame, item.current, item.autocontinue);

    // update TX
    ml_tx_update();

    //start timeout
    ros::NodeHandle nh;
    mission_up_timeout = nh.createTimer(ros::Duration(MISSION_ITEM_TIMEOUT_TIME), mission_up_item_timeout_callback, true, true);
}
/***************************************************************************/
void mission_up_item_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_lora::mavlink_lora_mission_ack msg;
        msg.drone_id = recorded_sysid;
        msg.result = 20;
        msg.result_text = mission_result_parser(20); //max retries error
        mission_ack_pub.publish(msg);

        //cancel command
        mission_up_index = -1;
        mission_up_count = 0;
        missionlist.clear();
        mission_uploading = false;

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    ROS_INFO_STREAM("MISSION ITEM TIMEOUT!");
    //if timeout triggers, resend last mission item
    ml_send_mission_item_int();
}
/***************************************************************************/
void mission_clear_all_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_lora::mavlink_lora_mission_ack msg;
        msg.drone_id = recorded_sysid;
        msg.result = 20;
        msg.result_text = mission_result_parser(20); //max retries error
        mission_ack_pub.publish(msg);

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    //if timeout triggers, resend
    ml_send_mission_clear_all();
}
/***************************************************************************/
void mission_up_count_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_lora::mavlink_lora_mission_ack msg;
        msg.drone_id = recorded_sysid;
        msg.result = 20;
        msg.result_text = mission_result_parser(20); //max retries error
        mission_ack_pub.publish(msg);

        //cancel command
        mission_uploading = false;

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    //if timeout triggers, resend count message
    ml_send_mission_count();
}
/***************************************************************************/
void mission_ack_timeout_callback(const ros::TimerEvent&)
{        
    //publish error on ack topic
    mavlink_lora::mavlink_lora_mission_ack msg;
    msg.drone_id = recorded_sysid;
    msg.result = 19;
    msg.result_text = mission_result_parser(19); //ack timeout error
    mission_ack_pub.publish(msg); 
    ROS_WARN_STREAM("Wait for acknowledge timed out");
}
/***************************************************************************/
void mission_up_partial_timeout_callback(const ros::TimerEvent&)
{
    //increment retries
    mission_retries++;

    //check if retries has been exceeded
    if (mission_retries > MISSION_MAX_RETRIES)
    {
        //publish error on ack topic
        mavlink_lora::mavlink_lora_mission_ack msg;
        msg.drone_id = recorded_sysid;
        msg.result = 20;
        msg.result_text = mission_result_parser(20); //max retries error
        mission_ack_pub.publish(msg);

        //cancel command
        mission_uploading = false;

        //reset retries
        mission_retries = 0;

        //debug
        ROS_INFO_STREAM("MAX RETRIES REACHED");

        return;
    }

    //if timeout triggers, resend count message
    ml_send_partial_list();
}
/***************************************************************************/
std::string mission_result_parser(uint8_t result)
{
    switch(result)
    {
        case 0:
            return "MAV_MISSION_ACCEPTED";
        case 1:
            return "MAV_MISSION_ERROR";
        case 2:
            return "MAV_MISSION_UNSUPPORTED_FRAME";
        case 3:
            return "MAV_MISSION_UNSUPPORTED";
        case 4:
            return "MAV_MISSION_NO_SPACE";
        case 5:
            return "MAV_MISSION_INVALID";
        case 6:
            return "MAV_MISSION_INVALID_PARAM1";
        case 7:
            return "MAV_MISSION_INVALID_PARAM2";
        case 8:
            return "MAV_MISSION_INVALID_PARAM3";
        case 9:
            return "MAV_MISSION_INVALID_PARAM4";
        case 10:
            return "MAV_MISSION_INVALID_PARAM5_X";
        case 11:
            return "MAV_MISSION_INVALID_PARAM6_Y";
        case 12:
            return "MAV_MISSION_INVALID_PARAM7";
        case 13:
            return "MAV_MISSION_INVALID_SEQUENCE";
        case 14:
            return "MAV_MISSION_DENIED";
        case 19:
            return "MAV_MISSION_ACK_TIMEOUT";
        case 20:
            return "MAV_MISSION_MAX_RETRIES"; //Custom error indicating aborting due to max retries with no success
        default:
            return "DIDN'T RECOGNIZE RESULT CODE";
    }
}
/************************* INTERFACE COMMANDS ******************************/
void ml_command_arm_disarm_callback(const std_msgs::Bool::ConstPtr& msg)
{
    //arm if true, disarm if false
    //send msg
    ml_send_command_long(MAVLINK_MSG_ID_COMPONENT_ARM_DISARM, msg->data, 0, 0, 0, 0, 0, 0);
}
void ml_command_start_mission_callback(const mavlink_lora::mavlink_lora_command_start_mission::ConstPtr& msg)
{
    //start mission, first and last item as parameter
    //send msg
    ml_send_command_long(MAVLINK_MSG_ID_MISSION_START, msg->first_item, msg->last_item, 0, 0, 0, 0, 0);
}
void ml_command_set_mode_callback(const mavlink_lora::mavlink_lora_command_set_mode::ConstPtr& msg)
{
    ml_send_command_long(MAVLINK_MSG_ID_DO_SET_MODE, msg->base_mode, msg->custom_main_mode, msg->custom_sub_mode, 0, 0, 0, 0);
}
/***************************** HEARTBEAT ***********************************/
void ml_send_heartbeat()
{
    //queue heartbeat
    ml_queue_msg_heartbeat();

    //update tx
    ml_tx_update();

    //update last sent
    heartbeat_msg_sent = ros::Time::now();
}

/***************************** OFFBOARD ************************************/
void ml_offboard_new_local_pos_sp_callback(const mavlink_lora::mavlink_lora_new_local_pos_sp::ConstPtr& msg)
{
    local_sp_x = msg->x;
    local_sp_y = msg->y;
    local_sp_z = msg->z;
    local_sp_yaw = msg->yaw;
    local_sp_yaw_rate = msg->yaw_rate;   
    local_sp_frame = msg->coordinate_frame;
    local_sp_typemask = 0x03F8; // ignore velocity and acceleration
}

void ml_offboard_new_local_vel_sp_callback(const mavlink_lora::mavlink_lora_new_local_vel_sp::ConstPtr& msg)
{
    local_sp_vx = msg->vx;
    local_sp_vy = msg->vy;
    local_sp_vz = msg->vz;
    local_sp_yaw = msg->yaw;
    local_sp_yaw_rate = msg->yaw_rate;   
    local_sp_frame = msg->coordinate_frame;
    local_sp_typemask = 0x03C7; // ignore position and acceleration
}

void ml_offboard_new_local_acc_sp_callback(const mavlink_lora::mavlink_lora_new_local_acc_sp::ConstPtr& msg)
{
    local_sp_ax = msg->ax;
    local_sp_ay = msg->ay;
    local_sp_az = msg->az;
    local_sp_yaw = msg->yaw;
    local_sp_yaw_rate = msg->yaw_rate;   
    local_sp_frame = msg->coordinate_frame;
    local_sp_typemask = 0x023F; // ignore position and velocity
}

void send_local_setpoint_callback(const ros::TimerEvent&)
{
    auto now = ros::Time::now();
    uint32_t time_boot_ms = now.sec*1000 + now.nsec/1000;
    //queue setpoint message
    ml_queue_msg_set_position_target_local_ned(
        time_boot_ms, 
        local_sp_frame, 
        local_sp_typemask, 
        local_sp_x, 
        local_sp_y, 
        local_sp_z, 
        local_sp_vx,
        local_sp_vy,
        local_sp_vz,
        local_sp_ax,
        local_sp_ay,
        local_sp_az,
        local_sp_yaw,
        local_sp_yaw_rate
        );

    //update tx
    ml_tx_update();
}

/******************************* MAIN **************************************/
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

	/* Interface subscribers */
    ros::Subscriber mission_upload_sub = n.subscribe("mavlink_interface/mission/mavlink_upload_mission", 1, ml_new_mission_callback);
    ros::Subscriber partial_mission_upload_sub = n.subscribe("mavlink_interface/mission/mavlink_upload_partial_mission", 1, ml_partial_mission_callback);
    ros::Subscriber mission_clear_all_sub = n.subscribe("mavlink_interface/mission/mavlink_clear_all", 1, ml_mission_clear_all_callback);
    ros::Subscriber command_arm_disarm_sub = n.subscribe("mavlink_interface/command/arm_disarm", 1, ml_command_arm_disarm_callback);
    ros::Subscriber command_start_mission_sub = n.subscribe("mavlink_interface/command/start_mission", 1, ml_command_start_mission_callback);
    ros::Subscriber command_set_mode_sub = n.subscribe("mavlink_interface/command/set_mode", 1, ml_command_set_mode_callback);
    ros::Subscriber local_pos_sp_sub = n.subscribe("mavlink_interface/offboard/new_pos_sp", 1, ml_offboard_new_local_pos_sp_callback);
    ros::Subscriber local_vel_sp_sub = n.subscribe("mavlink_interface/offboard/new_vel_sp", 1, ml_offboard_new_local_vel_sp_callback);
    ros::Subscriber local_acc_sp_sub = n.subscribe("mavlink_interface/offboard/new_acc_sp", 1, ml_offboard_new_local_acc_sp_callback);

    /* Interface publishers */
    mission_ack_pub = n.advertise<mavlink_lora::mavlink_lora_mission_ack>("mavlink_interface/mission/ack", 1);
    command_ack_pub = n.advertise<mavlink_lora::mavlink_lora_command_ack>("mavlink_interface/command/ack", 1);

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

    /* start local setpoint spam */
    // ros::Timer timer = n.createTimer(ros::Duration(0.125), send_local_setpoint_callback);

	/* ros main loop */
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		ros::spinOnce();

		/* check if it is time to send a status message */
		if (status_msg_sent + ros::Duration(1) <= ros::Time::now())
		    ml_send_status_msg();

		/* check if time to emit heartbeat */
		if (heartbeat_msg_sent + ros::Duration(0.2) <= ros::Time::now())
		    ml_send_heartbeat();

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


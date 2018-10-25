/***************************************************************************
# MavLink LoRa library
# MavLink long range communication library 
# Copyright (c) 2017-2018, Kjeld Jensen <kjen@mmmi.sdu.dk> <kj@kjen.dk>
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
# This library is based on the output from the MavLink generator
# http://www.mavlink.org which is released under the MIT license:
# https://opensource.org/licenses/MIT
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
2018-04-04 KJ Added unpacking of statustext
2018-04-10 KJ Significant rewrites in order to make it Arduino compatible
2018-06-11 KJ Added support for attitude
****************************************************************************/

#ifndef KJMAVLINK_H
#define KJMAVLINK_H

/***************************************************************************/
/* parameters */

#define RX_BUF_SIZE	16000 /* size of the receive buffer, depends on system and application */
#define PARAM_TOUT 2000 /* [ms] timeout for other msgs when exchanging parameters */
#define MISSION_TOUT 2000 /* [ms] timeout for other msgs when exchanging mission */

/* #define DROP_MSGS_WITH_ID {30, 32, 35, 36, 62, 109} */ /* AutoQuad long range default */

#define DROP_MSGS_WITH_ID {27, 29, 35, 36, 62, 65, 74, 116, 129, 137, 178, 182, 241} /* PX4 long range default */

#define SLOW_DOWN_MSGS_WITH_ID {0, 1, 2, 24, 30, 33, 42, 125, 152, 163, 165, 193}
#define SLOW_DOWN_TIMEOUTS {1000, 3000, 3000, 2000, 2000, 1000, 1000, 5000, 3000, 2000, 5000, 3000}

/***************************************************************************/
/* inludes */

#include <stdint.h>

/***************************************************************************/
/* defines */

/* serial stream buffers */
#define TX_BUF_SIZE	150

/* mavlink message content */
#define ML_NEW_PACKET_IDENT_V10	0xfe /* MavLink v1.0 */
#define ML_NEW_PACKET_IDENT_V20	0xfd /* MavLink v2.0 */

/* mavlink system id */
#define MAV_SYS_ID_ALL 0
#define MAV_SYS_ID_UA 1
#define MAV_SYS_ID_GCS 255

/* mavlink component id */
/*#define MAV_COMP_ID_ALL 0 */

/* mavlink message format */
#define ML_POS_IDENT 0
#define ML_POS_PAYLOAD_LEN 1
#define ML_POS_PACKET_SEQ 2
#define ML_POS_SYS_ID 3
#define ML_POS_COMP_ID 4
#define ML_POS_MSG_ID 5
#define ML_POS_PAYLOAD 6

/* mavlink message id's /*
/* needed because we are not including the official mavlink headers */

#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 8

#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31

#define MAVLINK_MSG_ID_PARAM_REQUEST_READ 20
#define MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN 20

#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST 21
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN 2

#define MAVLINK_MSG_ID_PARAM_VALUE 22
#define MAVLINK_MSG_ID_PARAM_VALUE_LEN 25

#define MAVLINK_MSG_ID_PARAM_SET 23
#define MAVLINK_MSG_ID_PARAM_SET_LEN 23

#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30

#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28

#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28

#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST 37
#define MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST_LEN 6

#define MAVLINK_MSG_ID_WRITE_PARTIAL_LIST 38
#define MAVLINK_MSG_ID_WRITE_PARTIAL_LIST_LEN 6

#define MAVLINK_MSG_ID_MISSION_ITEM 39
#define MAVLINK_MSG_ID_MISSION_ITEM_LEN 37

#define MAVLINK_MSG_ID_MISSION_ITEM_INT 73
#define MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN 37

#define MAVLINK_MSG_ID_MISSION_REQUEST 40
#define MAVLINK_MSG_ID_MISSION_REQUEST_LEN 4

#define MAVLINK_MSG_ID_MISSION_REQUEST_INT 51
#define MAVLINK_MSG_ID_MISSION_REQUEST_INT_LEN 4

#define MAVLINK_MSG_ID_MISSION_CURRENT 42
#define MAVLINK_MSG_ID_MISSION_CURRENT_LEN 2

#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN 2

#define MAVLINK_MSG_ID_MISSION_COUNT 44
#define MAVLINK_MSG_ID_MISSION_COUNT_LEN 4

#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL 45
#define MAVLINK_MSG_ID_MISSION_CLEAR_ALL_LEN 2

#define MAVLINK_MSG_ID_MISSION_ACK 47
#define MAVLINK_MSG_ID_MISSION_ACK_LEN 3

#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM 66
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM_LEN 6

#define MAVLINK_MSG_ID_STATUSTEXT 253
#define MAVLINK_MSG_ID_STATUSTEXT_LEN 51

#define MAVLINK_MSG_ID_COMMAND_LONG 76
#define MAVLINK_MSG_ID_COMMAND_LONG_LEN 33

#define MAVLINK_MSG_ID_COMMAND_ACK 77
#define MAVLINK_MSG_ID_COMMAND_ACK_LEN 3

#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED 84
#define MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN 53

#define MAVLINK_MSG_ID_COMPONENT_ARM_DISARM 400
#define MAVLINK_MSG_ID_MISSION_START 300
#define MAVLINK_MSG_ID_DO_SET_MODE 176


typedef struct __mavlink_sys_status_t {
 uint32_t onboard_control_sensors_present; /*< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
 uint32_t onboard_control_sensors_enabled; /*< Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
 uint32_t onboard_control_sensors_health; /*< Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR*/
 uint16_t load; /*< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000*/
 uint16_t voltage_battery; /*< Battery voltage, in millivolts (1 = 1 millivolt)*/
 int16_t current_battery; /*< Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current*/
 uint16_t drop_rate_comm; /*< Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
 uint16_t errors_comm; /*< Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
 uint16_t errors_count1; /*< Autopilot-specific errors*/
 uint16_t errors_count2; /*< Autopilot-specific errors*/
 uint16_t errors_count3; /*< Autopilot-specific errors*/
 uint16_t errors_count4; /*< Autopilot-specific errors*/
 int8_t battery_remaining; /*< Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery*/
} mavlink_sys_status_t;

typedef struct battery_status_t {
	unsigned short voltage; /* [mV] */
	char battery_remaining; /* [0;100] [%], -1 means invalid data */
} battery_status_t;

typedef struct __mavlink_attitude_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 float roll; /*< Roll angle (rad, -pi..+pi)*/
 float pitch; /*< Pitch angle (rad, -pi..+pi)*/
 float yaw; /*< Yaw angle (rad, -pi..+pi)*/
 float rollspeed; /*< Roll angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
 float yawspeed; /*< Yaw angular speed (rad/s)*/
} mavlink_attitude_t;

typedef struct __mavlink_gps_raw_int_t {
 uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
 int32_t lat; /*< Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7*/
 int32_t lon; /*< Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7*/
 int32_t alt; /*< Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.*/
 uint16_t eph; /*< GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t epv; /*< GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX*/
 uint16_t vel; /*< GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX*/
 uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
 uint8_t fix_type; /*< See the GPS_FIX_TYPE enum.*/
 uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
} mavlink_gps_raw_int_t;

typedef struct __mavlink_global_position_int_t {
 uint32_t time_boot_ms; /*< Timestamp (milliseconds since system boot)*/
 int32_t lat; /*< Latitude, expressed as degrees * 1E7*/
 int32_t lon; /*< Longitude, expressed as degrees * 1E7*/
 int32_t alt; /*< Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)*/
 int32_t relative_alt; /*< Altitude above ground in meters, expressed as * 1000 (millimeters)*/
 int16_t vx; /*< Ground X Speed (Latitude, positive north), expressed as m/s * 100*/
 int16_t vy; /*< Ground Y Speed (Longitude, positive east), expressed as m/s * 100*/
 int16_t vz; /*< Ground Z Speed (Altitude, positive down), expressed as m/s * 100*/
 uint16_t hdg; /*< Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
} mavlink_global_position_int_t;

typedef struct __mavlink_mission_item_t { /* mavlink/common/mavlink_msg_mission_item.h */
 float param1; /*< PARAM1, see MAV_CMD enum*/
 float param2; /*< PARAM2, see MAV_CMD enum*/
 float param3; /*< PARAM3, see MAV_CMD enum*/
 float param4; /*< PARAM4, see MAV_CMD enum*/
 float x; /*< PARAM5 / local: x position, global: latitude*/
 float y; /*< PARAM6 / y position: global: longitude*/
 float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
 uint16_t seq; /*< Sequence*/
 uint16_t command; /*< The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs*/
 uint8_t target_system; /*< System ID*/
 uint8_t target_component; /*< Component ID*/
 uint8_t frame; /*< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h*/
 uint8_t current; /*< false:0, true:1*/
 uint8_t autocontinue; /*< autocontinue to next wp*/
} mavlink_mission_item_t;

typedef struct __mavlink_mission_item_int_t { /* mavlink/common/mavlink_msg_mission_item_int.h */
    float param1; /*< PARAM1, see MAV_CMD enum*/
    float param2; /*< PARAM2, see MAV_CMD enum*/
    float param3; /*< PARAM3, see MAV_CMD enum*/
    float param4; /*< PARAM4, see MAV_CMD enum*/
    uint32_t x; /*< PARAM5 / local: x position, global: latitude*/
    uint32_t y; /*< PARAM6 / y position: global: longitude*/
    float z; /*< PARAM7 / z position: global: altitude (relative or absolute, depending on frame.*/
    uint16_t seq; /*< Sequence*/
    uint16_t command; /*< The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t frame; /*< The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h*/
    uint8_t current; /*< false:0, true:1*/
    uint8_t autocontinue; /*< autocontinue to next wp*/
} mavlink_mission_item_int_t;

typedef struct __mavlink_mission_request_int_t { /* mavlink/common/mavlink_msg_mission_request_int.h */
    uint16_t seq; /*< Sequence*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
} mavlink_mission_request_int_t;

typedef struct __mavlink_mission_count_t { /* mavlink/common/mavlink_msg_mission_count.h */
    uint16_t count; /*< number of mission items in the sequence*/
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
} mavlink_mission_count_t;

typedef struct __mavlink_mission_clear_all_t { /* mavlink/common/mavlink_msg_mission_clear_all */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
} mavlink_mission_clear_all_t;

typedef struct __mavlink_mission_ack_T { /* mavlink/common/mavlink_msg_mission_ack.h */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t type; /*< Mission Result, check enum: MAV_MISSION_RESULT */
} mavlink_mission_ack_t;

typedef struct __mavlink_statustext_t {
 uint8_t severity; /*< Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.*/
 char text[50]; /*< Status text message, without null termination character*/
} mavlink_statustext_t;

typedef struct __mavlink_command_long_t { /* mavlink/common/COMMAND_LONG */
    uint16_t command; /*<Command ID */
    float param1; /* < parameter 1 */
    float param2; /* < parameter 2 */
    float param3; /* < parameter 3 */
    float param4; /* < parameter 4 */
    float param5; /* < parameter 5 */
    float param6; /* < parameter 6 */
    float param7; /* < parameter 7 */
    uint8_t target_system; /*< System ID*/
    uint8_t target_component; /*< Component ID*/
    uint8_t confirmation; /*< Confirmation, 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command) */
} mavlink_command_long_t;

typedef struct __mavlink_command_ack_t { /* mavlink/common/COMMAND_ACK */
 uint16_t command; /*<Command ID (of acknowledged command)*/
 uint8_t result; /*<Result of command */
} mavlink_command_ack_t;

/***************************************************************************/
/* global variables */

extern unsigned char txbuf[TX_BUF_SIZE];
extern unsigned char rxbuf_filtered[RX_BUF_SIZE];
extern short txbuf_cnt;
extern short rxbuf_filtered_cnt;

/***************************************************************************/
/* function prototypes */

void ml_init(void);
uint8_t ml_recorded_sys_id(void);
unsigned long ml_messages_sent(void);
unsigned long ml_messages_received(void);
unsigned long ml_messages_crc_error(void);
void ml_set_monitor_all(void);
short ml_rx_update(unsigned long ms, unsigned char *buf, short buf_cnt);
mavlink_sys_status_t ml_unpack_msg_sys_status (unsigned char *payload);
battery_status_t ml_unpack_msg_battery_status (unsigned char *payload);
mavlink_attitude_t ml_unpack_msg_attitude (unsigned char *payload);
mavlink_gps_raw_int_t ml_unpack_msg_gps_raw_int (unsigned char *payload);
mavlink_global_position_int_t ml_unpack_msg_global_position_int (unsigned char *payload);
unsigned short ml_unpack_msg_mission_count (unsigned char *payload);
mavlink_mission_item_t ml_unpack_msg_mission_item (unsigned char *payload);
mavlink_mission_item_int_t ml_unpack_msg_mission_item_int (unsigned char *payload);
mavlink_mission_request_int_t ml_unpack_msg_mission_request_int (unsigned char *payload);
mavlink_mission_ack_t ml_unpack_msg_mission_ack (unsigned char *payload);
mavlink_statustext_t ml_unpack_msg_statustext (unsigned char *payload);
mavlink_command_ack_t ml_unpack_msg_command_ack (unsigned char *payload);
void ml_queue_msg_generic (unsigned char sys_id, unsigned char comp_id, unsigned char msg_id, unsigned char payload_len, unsigned char *payload);
void ml_queue_msg_param_request_read (char *param_id);
void ml_queue_msg_param_request_list (void);
void ml_queue_msg_param_set (char *param_id, float param_value);
void ml_queue_msg_mission_request (unsigned short seq);
void ml_queue_msg_mission_request_list (void);
void ml_queue_msg_mission_ack (void);
void ml_queue_msg_mission_count (unsigned short count);
void ml_queue_msg_write_partial_list (uint16_t start_idx, uint16_t end_idx);
void ml_queue_msg_mission_clear_all (void);
void ml_queue_msg_mission_item_int (float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, unsigned short seq, unsigned short command, unsigned char frame, unsigned char current, unsigned char autocontinue);
void ml_queue_msg_command_long (unsigned short cmd_id, float param1, float param2, float param3, float param4, float param5, float param6, float param7, unsigned int confirmation);
void ml_queue_msg_heartbeat (void);
void ml_queue_msg_set_position_target_local_ned(uint32_t time_boot_ms, uint8_t frame, uint16_t type_mask, float x, float y, float z, float vx, float vy, float vz, float ax, float ay, float az, float yaw, float yaw_rate);

/***************************************************************************/
/* callback functions */

extern void ml_parse_msg(unsigned char *msg);
extern void ml_tx_update (void);

/***************************************************************************/
#endif 


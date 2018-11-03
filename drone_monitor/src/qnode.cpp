/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/drone_monitor/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drone_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/
bool operator!=(const gcs::GPS& op1, const gcs::GPS& op2){
    return op1.latitude != op2.latitude &&op1.longitude != op2.longitude && op1.altitude &&op2.altitude;
}

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"drone_monitor");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    n.subscribe<gcs::DroneInfo>("/drone_handler/DroneInfo",1,&QNode::handle_DroneInfo,this);
    n.subscribe<gcs::NiceInfo>("/drone_handler/NiceInfo",1,&QNode::handle_NiceInfo,this);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::handle_NiceInfo(gcs::NiceInfo msg){
    aDrone* theDrone= &this->_Drones[int(msg.drone_id)];
    theDrone->drone_ID = msg.drone_id;

    if(theDrone->uptime != msg.up_time){
        theDrone->uptime = msg.up_time;
        Q_EMIT sig_uptime(theDrone->uptime);
    }
    if(theDrone->drone_handler_state != msg.drone_handler_state){
        theDrone->drone_handler_state = msg.drone_handler_state;
        //TODO
    }
    if(theDrone->autoPilot != msg.autopilot){
        theDrone->autoPilot = msg.autopilot;
        Q_EMIT sig_autoPilot(theDrone->autoPilot);
    }
    if(theDrone->mavlinkState != msg.mav_state){
        theDrone->mavlinkState = msg.mav_state;
        Q_EMIT sig_mavlinkState(theDrone->mavlinkState);
    }
    if(theDrone->mavlinkType != msg.mav_type){
        theDrone->mavlinkType = msg.mav_type;
        Q_EMIT sig_mavlinkType(theDrone->mavlinkType);
    }
    if(theDrone->mainFlightMode != msg.main_flightmode){
        theDrone->mainFlightMode = msg.main_flightmode;
        Q_EMIT sig_mainFlightmode(theDrone->mainFlightMode);
    }
    if(theDrone->subFlightmode !=msg.sub_flightmode){
        theDrone->subFlightmode= msg.sub_flightmode;
        Q_EMIT sig_subFlightmode(theDrone->subFlightmode);
    }
    if(theDrone->msg_sent_gcs != msg.msg_sent_gcs){
        theDrone->msg_sent_gcs = msg.msg_sent_gcs;
        //TODO
    }
    if(theDrone->msg_received_gcs != msg.msg_received_gcs){
        theDrone->msg_received_gcs = msg.msg_received_gcs;
        //TODO
    }
    if(theDrone->msg_dropped_gcs != msg.msg_dropped_gcs){
        theDrone->msg_dropped_gcs = msg.msg_dropped_gcs;
        //TODO
    }
    if(theDrone->msg_dropped_uas != msg.msg_dropped_uas){
        theDrone->msg_dropped_uas = msg.msg_dropped_uas;
        //TODO
    }
    //uint16 active_waypoint_idx	# On drone index
    //uint16 active_mission_len	# On drone mission length

    if(theDrone->armed != msg.armed){
        theDrone->armed = msg.armed;
        Q_EMIT sig_armed(theDrone->armed);
    }
    if(theDrone->manual != msg.manual_input &&
            theDrone->simulation != msg.hil_simulation &&
            theDrone->stabilized != msg.stabilized_mode &&
            theDrone->guided != msg.guided_mode &&
            theDrone->autoM != msg.auto_mode &&
            theDrone->test != msg.test_mode &&
            theDrone->custom != msg.custom_mode)
    {
        theDrone->manual = msg.manual_input;
        theDrone->simulation = msg.hil_simulation;
        theDrone->stabilized = msg.stabilized_mode;
        theDrone->guided = msg.guided_mode;
        theDrone->autoM = msg.auto_mode;
        theDrone->test = msg.test_mode;
        theDrone->custom = msg.custom_mode;
        Q_EMIT sig_otherFlightmode(theDrone->manual,theDrone->simulation,theDrone->stabilized,theDrone->guided, theDrone->autoM,theDrone->test,theDrone->custom);
    }
    if(theDrone->RPY.size() < 3){
        theDrone->RPY = vector<double>(3);
    }
    if(theDrone->RPY[0] != msg.RPY[0]&&
            theDrone->RPY[1] != msg.RPY[1] &&
            theDrone->RPY[2] != msg.RPY[2])
    {
        theDrone->RPY[0] = msg.RPY[0];
        theDrone->RPY[1] = msg.RPY[1];
        theDrone->RPY[2] = msg.RPY[2];
        Q_EMIT sig_RPY(theDrone->RPY);
    }
    if(theDrone->climb_rate != msg.climb_rate){
        theDrone->climb_rate = msg.climb_rate;
        //TODO
    }
    if(theDrone->throttle != msg.throttle){
        theDrone->throttle = msg.throttle;
        //TODO
    }
    if(theDrone->home != msg.home){
        theDrone->home = msg.home;
        //TODO
    }
}
void QNode::handle_DroneInfo(gcs::DroneInfo msg){
    aDrone* theDrone= &_Drones[int(msg.drone_id)];
    theDrone->drone_ID = msg.drone_id;
    if(theDrone->armed != msg.armed){
        theDrone->armed = msg.armed;
        Q_EMIT sig_armed(theDrone->armed);
    }
    if(theDrone->status != msg.status){
        theDrone->status = msg.status;
        Q_EMIT sig_status(theDrone->status);
    }
    if(theDrone->SOC != msg.battery_SOC){
        theDrone->SOC = msg.battery_SOC;
        Q_EMIT sig_battery(theDrone->SOC);
    }
    /*if(theDrone->time != msg.GPS_timestamp){
        theDrone->time = msg.GPS_timestamp;
        sig_battery(theDrone->time);
    }*/
    if(theDrone->heading != msg.heading){
        theDrone->heading = msg.heading;
        Q_EMIT sig_heading(theDrone->heading);
    }
    if(theDrone->position != msg.position){
        theDrone->position = msg.position;
        Q_EMIT sig_position(theDrone->position);
    }
    if(theDrone->height != msg.relative_alt){
        theDrone->height = msg.relative_alt;
        Q_EMIT sig_Height(theDrone->height);
    }
    if(theDrone->velocity != msg.ground_speed){
        theDrone->velocity = msg.ground_speed;
        Q_EMIT sig_velocity(theDrone->velocity);
    }
}
}  // namespace drone_monitor

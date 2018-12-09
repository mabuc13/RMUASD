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
#include <iostream>
#include <string>
#include "../include/drone_monitor/qnode.hpp"
#include <ctime>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}


namespace drone_monitor {

/*****************************************************************************
** Implementation
*****************************************************************************/
bool operator!=(const gcs::GPS& op1, const gcs::GPS& op2){
    return op1.latitude != op2.latitude || op1.longitude != op2.longitude || op1.altitude &&op2.altitude;
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
    this->_droneInfo_sub = n.subscribe<gcs::DroneInfo>("/drone_handler/DroneInfo",1,&QNode::handle_DroneInfo,this);
    this->_niceInfo_sub = n.subscribe<gcs::NiceInfo>("/drone_handler/NiceInfo",1,&QNode::handle_NiceInfo,this);
    this->_ETA_sub = n.subscribe<gcs::DroneSingleValue>("/gcs/ETA",1,&QNode::handle_ETA,this);
    this->_TelemetryStatus_sub = n.subscribe<telemetry::telemetry_statustext>("/telemetry/statustext",10,&QNode::handle_telemetryStatus,this);
    this->_JobState_sub = n.subscribe<gcs::DroneSingleValue>("/gcs/JobState",10, &QNode::handle_JobState,this);
    this->_NodeStat_sub = n.subscribe<node_monitor::nodeOkList>("/node_monitor/node_list", 10,&QNode::handle_NodeStat, this);
    this->_Node_monitor_heartbeat_sub = n.subscribe<node_monitor::heartbeat>("/node_monitor/Heartbeat",10,&QNode::handle_NodeMonitorHeart,this);


    std::cout << "Ros Monitor started" << std::endl << std::flush;

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;

    int state = -1,last_state = -1;
    while ( ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
        if(ros::Time::now().toSec()> ((1/_node_monitor_rate)*4+_node_monitor_time.toSec())){
            state = node_monitor::nodeOk::none_responsive;
        }else if(ros::Time::now().toSec()> ((1/_node_monitor_rate)*2+_node_monitor_time.toSec())){
            state = node_monitor::nodeOk::late;
        }else{
            state = node_monitor::nodeOk::fine;
        }
        //std::cout << "before Last Heard from: " << std::time(NULL)- this->_Drones[1].last_heard_from << endl;
        if(std::time(NULL)-this->_Drones[1].last_heard_from>8){
            Q_EMIT sig_nodeState("Drone1",node_monitor::heartbeat::nothing,node_monitor::nodeOk::none_responsive);
        }else if(std::time(NULL)-this->_Drones[1].last_heard_from>4){
            Q_EMIT sig_nodeState("Drone1",node_monitor::heartbeat::nothing,node_monitor::nodeOk::late);
        }else{
            Q_EMIT sig_nodeState("Drone1",node_monitor::heartbeat::nothing,node_monitor::nodeOk::fine);
        }
        if(last_state!= state){
            Q_EMIT sig_nodeState("node_monitor",_node_monitor_severity,state);
            last_state = state;
        }

	}
    Q_EMIT rosShutdown();
}


void QNode::setCurrentDrone(int drone_id){
    try{
        this->_Drones.at(drone_id);
        this->_currentDrone = drone_id;
    }catch (const std::out_of_range& oor) {
        std::cerr << "Out of Range error: " << oor.what() << '\n';
    }
}
void QNode::handle_NodeMonitorHeart(node_monitor::heartbeat msg){
    _node_monitor_time = msg.header.stamp;
    _node_monitor_rate = msg.rate;
    
    if(msg.text.length()> 0 && !(msg.text == _last_text_msg)){
        string severity = "";
        if(msg.severity == node_monitor::heartbeat::info){
            severity = "info";
            msg.severity = 6;
        }else if(msg.severity == node_monitor::heartbeat::warning){
            severity = "warning";
            msg.severity=5;
        }else if(msg.severity == node_monitor::heartbeat::error){
            severity = "error";
            msg.severity=2;
        }else if(msg.severity == node_monitor::heartbeat::critical_error){
            severity = "critical";
            msg.severity=1;
        }else if(msg.severity == node_monitor::heartbeat::fatal_error){
            severity = "fatal";
            msg.severity=0;
        }
        _last_text_msg = msg.text;
        string text ="["+severity+"][" +patch::to_string(ros::Time::now().sec) +"]: " + msg.text+"\n";
        Q_EMIT sig_telemetryStatus(msg.severity,text.c_str());
    }
    if(msg.severity != _node_monitor_severity){
        _node_monitor_severity = 1;
        Q_EMIT sig_nodeState("node_monitor",_node_monitor_severity,node_monitor::nodeOk::fine);
    }
}
void QNode::handle_NodeStat(node_monitor::nodeOkList msg){
    //cout << "Message: "<< endl<<flush;
    for(size_t i = 0; i < msg.Nodes.size(); i++){
        aNode* theNode = &_Nodes[msg.Nodes[i].name];
        //cout << "Name: " << msg.Nodes[i].name
        if(theNode->res_OK != msg.Nodes[i].ok ||
           theNode->state_OK != msg.Nodes[i].nodeState ||
           !theNode->isInit)
        {
            theNode->isInit = true;
            //cout << "DataParsed: " << msg.Nodes[i].name << " - " << int(msg.Nodes[i].nodeState) << " - " << int(msg.Nodes[i].ok) << endl;
            theNode->res_OK = msg.Nodes[i].ok;
            theNode->state_OK = msg.Nodes[i].nodeState;
            Q_EMIT sig_nodeState(msg.Nodes[i].name.c_str(),theNode->state_OK,theNode->res_OK);
        }
    }
}

void QNode::handle_JobState(gcs::DroneSingleValue msg){
    aDrone* theDrone= &this->_Drones[int(msg.drone_id)];
    theDrone->drone_ID = msg.drone_id;
    if(theDrone->gcsJobState != msg.value){
        theDrone->gcsJobState = msg.value;
        Q_EMIT sig_gcsJobState(theDrone->gcsJobState,msg.text.c_str());
    }
}
void QNode::handle_telemetryStatus(telemetry::telemetry_statustext msg){
    string text ="["+msg.severity+"][" +patch::to_string(ros::Time::now().sec) +"]: " + msg.text+"\n";
    Q_EMIT sig_telemetryStatus(msg.severity_level,text.c_str());
}
void QNode::handle_ETA(gcs::DroneSingleValue msg){
    aDrone* theDrone= &this->_Drones[int(msg.drone_id)];
    theDrone->drone_ID = msg.drone_id;
    if(theDrone->ETA != msg.value){
        theDrone->ETA = msg.value;
        Q_EMIT sig_ETA(theDrone->ETA-std::time(NULL));
    }
}
void QNode::handle_NiceInfo(gcs::NiceInfo msg){
    aDrone* theDrone= &this->_Drones[int(msg.drone_id)];
    theDrone->drone_ID = msg.drone_id;

    if(theDrone->uptime != msg.up_time){
        theDrone->uptime = msg.up_time;
        theDrone->last_heard_from = std::time(NULL);
        Q_EMIT sig_uptime(theDrone->uptime);
    }
    if(theDrone->drone_handler_state != msg.drone_handler_state){
        theDrone->drone_handler_state = msg.drone_handler_state;
        Q_EMIT sig_droneHandlerState(QString(theDrone->drone_handler_state.c_str()));
    }
    if(theDrone->autoPilot != msg.autopilot){
        theDrone->autoPilot = msg.autopilot;
        Q_EMIT sig_autoPilot(theDrone->autoPilot.c_str());
    }
    if(theDrone->mavlinkState != msg.mav_state){
        theDrone->mavlinkState = msg.mav_state;
        Q_EMIT sig_mavlinkState(theDrone->mavlinkState.c_str());
    }
    if(theDrone->mavlinkType != msg.mav_type){
        theDrone->mavlinkType = msg.mav_type;
        Q_EMIT sig_mavlinkType(theDrone->mavlinkType.c_str());
    }
    if(theDrone->mainFlightMode != msg.main_flightmode){
        theDrone->mainFlightMode = msg.main_flightmode;
        Q_EMIT sig_mainFlightmode(theDrone->mainFlightMode.c_str());
    }
    if(theDrone->subFlightmode !=msg.sub_flightmode){
        theDrone->subFlightmode= msg.sub_flightmode;
        Q_EMIT sig_subFlightmode(theDrone->subFlightmode.c_str());
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
    msg.active_waypoint_idx++;
    if(theDrone->drone_wayPoints !=msg.active_waypoint_idx	||
            theDrone->drone_missionLength != msg.active_mission_len){
        theDrone->drone_wayPoints = msg.active_waypoint_idx;
        theDrone->drone_missionLength = msg.active_mission_len;
        Q_EMIT sig_droneMission(theDrone->drone_wayPoints,theDrone->drone_missionLength);
    }
    //uint16 active_mission_len	# On drone mission length

    if(theDrone->armed != msg.armed){
        theDrone->armed = msg.armed;
        Q_EMIT sig_armed(theDrone->armed);
    }
    if(theDrone->manual != msg.manual_input ||
            theDrone->simulation != msg.hil_simulation ||
            theDrone->stabilized != msg.stabilized_mode ||
            theDrone->guided != msg.guided_mode ||
            theDrone->autoM != msg.auto_mode ||
            theDrone->test != msg.test_mode ||
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
    if(theDrone->RPY[0] != msg.RPY[0] ||
            theDrone->RPY[1] != msg.RPY[1] ||
            theDrone->RPY[2] != msg.RPY[2])
    {
        theDrone->RPY[0] = msg.RPY[0];
        theDrone->RPY[1] = msg.RPY[1];
        theDrone->RPY[2] = msg.RPY[2];
        Q_EMIT sig_RPY(theDrone->RPY[0],theDrone->RPY[1],theDrone->RPY[2]);
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
    if(theDrone->SOC != msg.battery_SOC){
        theDrone->SOC = msg.battery_SOC;
        Q_EMIT sig_battery(theDrone->SOC);
    }
    if(theDrone->GPStime != msg.GPS_timestamp){
        theDrone->GPStime = msg.GPS_timestamp;
        sig_time(theDrone->GPStime);
    }
    if(theDrone->heading != msg.heading){
        theDrone->heading = msg.heading;
        Q_EMIT sig_heading(theDrone->heading);
    }
    if(theDrone->position != msg.position){
        theDrone->position = msg.position;
        Q_EMIT sig_position(theDrone->position.altitude,
                            theDrone->position.longitude,
                            theDrone->position.latitude);
    }
    if(theDrone->height != msg.relative_alt){
        theDrone->height = msg.relative_alt;
        Q_EMIT sig_Height(theDrone->height);
    }
    if(theDrone->velocity != msg.ground_speed){
        theDrone->velocity = msg.ground_speed;
        Q_EMIT sig_velocity(theDrone->velocity);
    }
    if(theDrone->next_waypoint != msg.next_waypoint){
        theDrone->next_waypoint = msg.next_waypoint;
        Q_EMIT sig_nextWayPoint(theDrone->next_waypoint.altitude,
                                theDrone->next_waypoint.longitude,
                                theDrone->next_waypoint.latitude);
    }
    msg.mission_index++;
    if(theDrone->missionIndex != msg.mission_index){
        theDrone->missionIndex = msg.mission_index;
        Q_EMIT sig_missionIndex(theDrone->missionIndex);
    }
    if(theDrone->mission_length != msg.mission_length){
        theDrone->mission_length = msg.mission_length;
        Q_EMIT sig_missionLength(theDrone->mission_length);
    }
    if(theDrone->status != msg.status){
        theDrone->status = msg.status;
        Q_EMIT sig_status(theDrone->status);
    }
}
}  // namespace drone_monitor

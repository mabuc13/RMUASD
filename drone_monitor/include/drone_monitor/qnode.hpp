/**
 * @file /include/drone_monitor/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef drone_monitor_QNODE_HPP_
#define drone_monitor_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
	#include <ros/ros.h>
	#include <gcs/GPS.h>
	#include <gcs/DroneInfo.h>
	#include <gcs/NiceInfo.h>
	#include <gcs/DroneSingleValue.h>
	#include <telemetry/telemetry_statustext.h>
	#include <node_monitor/nodeOkList.h>
	#include <node_monitor/heartbeat.h>
	#include <node_monitor/nodeOk.h>
#endif

#include <string>
#include <QThread>
#include <QString>
#include <map>


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace drone_monitor {

/*****************************************************************************
** Class
*****************************************************************************/
struct aDrone{
	int drone_ID;
	bool armed;
	int status;
	double SOC;
	string time;
	double uptime;
	vector<double> RPY;
	double heading;
	gcs::GPS position;
	gcs::GPS next_waypoint;
	double height;
	double velocity;
	string autoPilot;
	string subFlightmode;
	string mainFlightMode;
	bool manual;
	bool simulation;
	bool stabilized;
	bool guided;
	bool autoM;
	bool test;
	bool custom;
	string mavlinkState;
	string mavlinkType;
	string drone_handler_state;
	size_t msg_sent_gcs;
	size_t msg_received_gcs;
	size_t msg_dropped_gcs;
	size_t msg_dropped_uas;
	double climb_rate;
	size_t throttle;
	gcs::GPS home;
	int missionIndex;
	int drone_wayPoints;
	int drone_missionLength;
	size_t ETA;
	int gcsJobState;
};
struct aNode{
	int res_OK;
	int state_OK;
	bool isInit;
};


class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	void setCurrentDrone(int drone_id);

Q_SIGNALS:
	void rosShutdown();
	void sig_armed(bool isArmed);
	void sig_status(int status);

	void sig_battery(double SOC);
	void sig_time(QString time);
	void sig_uptime(double sec);

	void sig_RPY(double R,double P,double Y);
	void sig_heading(double heading);
	void sig_position(double alt,double lon,double lat);
	void sig_Height(double height);
	void sig_velocity(double velocity);

	void sig_nextWayPoint(double alt,double lon,double lat);
	void sig_missionLength(int len);
	void sig_missionIndex(int len);
	void sig_droneMission(int index, int len);
	void sig_ETA(int sec);

	void sig_autoPilot(QString text);
	void sig_subFlightmode(QString text);
	void sig_mainFlightmode(QString text);
	void sig_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom);
	void sig_mavlinkState(QString state);
	void sig_mavlinkType(QString type);
	void sig_droneHandlerState(QString text);
	void sig_telemetryStatus(int severity, QString text);
	void sig_gcsJobState(int state,QString text);

	void sig_nodeState(QString name,int ownStatus,int response);

private:
	int init_argc;
	char** init_argv;
	ros::Subscriber _niceInfo_sub;
	ros::Subscriber _droneInfo_sub;
	ros::Subscriber _ETA_sub;
	ros::Subscriber _TelemetryStatus_sub;
	ros::Subscriber _JobState_sub;
	ros::Subscriber _NodeStat_sub;
	ros::Subscriber _Node_monitor_heartbeat_sub;

	void handle_NiceInfo(gcs::NiceInfo msg);
	void handle_DroneInfo(gcs::DroneInfo msg);
	void handle_ETA(gcs::DroneSingleValue msg);
	void handle_telemetryStatus(telemetry::telemetry_statustext msg);
	void handle_JobState(gcs::DroneSingleValue msg);
	void handle_NodeStat(node_monitor::nodeOkList msg);
	void handle_NodeMonitorHeart(node_monitor::heartbeat msg);

	map<int,aDrone> _Drones;
	map<string,aNode> _Nodes;
	int _currentDrone;
};

}  // namespace drone_monitor

#endif /* drone_monitor_QNODE_HPP_ */

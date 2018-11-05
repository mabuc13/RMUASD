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
#endif

#include <string>
#include <QThread>
#include <QStringListModel>
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
};

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	void close();

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
	void sig_armed(bool isArmed);
	void sig_status(int status);

	void sig_battery(double SOC);
	void sig_time(string time);
	void sig_uptime(double sec);

	void sig_RPY(vector<double> RPY);
	void sig_heading(double heading);
	void sig_position(double alt,double lon,double lat);
	void sig_nextWayPoint(double alt,double lon,double lat);
	void sig_Height(double height);
	void sig_velocity(double velocity);

	void sig_autoPilot(string text);
	void sig_subFlightmode(string text);
	void sig_mainFlightmode(string text);
	void sig_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom);
	void sig_mavlinkState(string state);
	void sig_mavlinkType(string type);

private:
	int init_argc;
	char** init_argv;
	ros::Subscriber _niceInfo_sub;
	ros::Subscriber _droneInfo_sub;

	void handle_NiceInfo(gcs::NiceInfo msg);
	void handle_DroneInfo(gcs::DroneInfo msg);

	map<int,aDrone> _Drones;
	int _currentDrone;

	bool closeDown;
};

}  // namespace drone_monitor

#endif /* drone_monitor_QNODE_HPP_ */

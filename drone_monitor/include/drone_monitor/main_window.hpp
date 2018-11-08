/**
 * @file /include/drone_monitor/main_window.hpp
 *
 * @brief Qt based gui for drone_monitor.
 *
 * @date November 2010
 **/
#ifndef drone_monitor_MAIN_WINDOW_H
#define drone_monitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace drone_monitor {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/


    /******************************************
    ** Manual connections
    *******************************************/
	void set_armed(bool isArmed);
	void set_status(int status);

	void set_battery(double SOC);
	void set_time(QString time);
	void set_uptime(double sec);

	void set_RPY(double R, double P, double Y);
	void set_heading(double heading);
	void set_position(double altitude,double longitude, double latitude);
	void set_Height(double height);
	void set_velocity(double velocity);

	void set_nextWayPoint(double altitude,double longitude, double latitude);
	void set_missionIndex(int len);
	void set_missionLength(int len);
	void set_droneMission(int index, int len);
	void set_ETA(int sec);

	void set_autoPilot(QString text);
	void set_subFlightmode(QString text);
	void set_mainFlightmode(QString text);
	void set_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom);
	void set_mavlinkState(QString state);
	void set_mavlinkType(QString type);
	void set_droneHandlerState(QString text);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace drone_monitor

#endif // drone_monitor_MAIN_WINDOW_H

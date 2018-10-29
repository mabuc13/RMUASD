/**
 * @file /include/dronemonitor/main_window.hpp
 *
 * @brief Qt based gui for dronemonitor.
 *
 * @date November 2010
 **/
#ifndef dronemonitor_MAIN_WINDOW_H
#define dronemonitor_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include <gcs/GPS.h>
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
namespace dronemonitor {

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
	void set_armed(bool isArmed);
	void set_status(int status);

	void set_battery(double SOC);
	void set_time(string time);
	void set_uptime(double sec);

	void set_RPY(vector<double> RPY);
	void set_heading(double heading);
	void set_position(gcs::GPS pos);
	void set_Height(double height);
	void set_velocity(double velocity);

	void set_autoPilot(string text);
	void set_subFlightmode(string text);
	void set_mainFlightmode(string text);
	void set_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom);
	void set_mavlinkState(string state);
	void set_mavlinkType(string type);


private:
	Ui::MainWindowDesign* ui;
};

}  // namespace dronemonitor

#endif // dronemonitor_MAIN_WINDOW_H

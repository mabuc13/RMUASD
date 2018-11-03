/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/drone_monitor/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace drone_monitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(sig_armed(bool)), this, SLOT(set_armed(bool)));
    QObject::connect(&qnode, SIGNAL(sig_status(int)), this, SLOT(set_status(int)));
    QObject::connect(&qnode, SIGNAL(sig_battery(double)), this, SLOT(set_battery(double)));
    QObject::connect(&qnode, SIGNAL(sig_time(string)), this, SLOT(set_time(string)));
    QObject::connect(&qnode, SIGNAL(sig_uptime(double)), this, SLOT(set_uptime(double)));
    QObject::connect(&qnode, SIGNAL(sig_RPY(vector<double>)), this, SLOT(set_RPY(vector<double>)));
    QObject::connect(&qnode, SIGNAL(sig_heading(double)), this, SLOT(set_heading(double)));
    QObject::connect(&qnode, SIGNAL(sig_position(gcs::GPS)), this, SLOT(set_position(gcs::GPS)));
    QObject::connect(&qnode, SIGNAL(sig_Height(double)), this, SLOT(set_Height(double)));
    QObject::connect(&qnode, SIGNAL(sig_velocity(double)), this, SLOT(set_velocity(double)));
    QObject::connect(&qnode, SIGNAL(sig_autoPilot(string)), this, SLOT(set_autoPilot(string)));
    QObject::connect(&qnode, SIGNAL(sig_subFlightmode(string)), this, SLOT(set_subFlightmode(string)));
    QObject::connect(&qnode, SIGNAL(sig_mainFlightmode(string)), this, SLOT(set_mainFlightmode(string)));
    QObject::connect(&qnode, SIGNAL(sig_otherFlightmode(bool,bool,bool,bool,bool,bool,bool)), this, SLOT(set_otherFlightmode(bool,bool,bool,bool,bool,bool,bool)));
    QObject::connect(&qnode,SIGNAL(sig_mavlinkState(string)),this,SLOT(set_mavlinkState(string)));
    QObject::connect(&qnode,SIGNAL(sig_mavlinkType(string)),this,SLOT(set_mavlinkType(string)));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

void MainWindow::set_armed(bool isArmed){
    if(isArmed){
        ui.label_armed->setText(QString("<font color='green'>ARMED</font>"));
    }else{
        ui.label_armed->setText(QString("<font color='red'>ARMED</font>"));
    }

}
void MainWindow::set_status(int status){
    if(status == 1){
        ui.label_status->setText(QString("Running Mission"));
    }else if(status == 2){
        ui.label_status->setText(QString("On Ground"));
    }else if(status == 3){
        ui.label_status->setText(QString("On hold"));
    }
}
void MainWindow::set_battery(double SOC){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(SOC,'f',2));
    text.push_back(QString("%"));
    ui.label_battery->setText(text);

}
void MainWindow::set_time(string time){
    ui.label_time->setText(time.c_str());
}
void MainWindow::set_uptime(double sec){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(sec,'f',2));
    text.push_back(QString("s"));
    ui.label_uptime->setText(text);
}
void MainWindow::set_RPY(vector<double> RPY){

}
void MainWindow::set_heading(double heading){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(heading,'f',2));
    //text.push_back(QString("s"));
    ui.label_heading->setText(text);
}
void MainWindow::set_position(gcs::GPS pos){

}
void MainWindow::set_Height(double height){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(height,'f',1));
    text.push_back(QString("m"));
    ui.label_height->setText(text);
}
void MainWindow::set_velocity(double velocity){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(velocity,'f',2));
    text.push_back(QString("m/s"));
    ui.label_velocity->setText(text);
}
void MainWindow::set_autoPilot(string text){
    ui.label_autoPilot->setText(text.c_str());
}
void MainWindow::set_subFlightmode(string text){
    ui.label_filghtmode_sub->setText(text.c_str());

}
void MainWindow::set_mainFlightmode(string text){
    ui.label_flightmode_main->setText(text.c_str());
}
void MainWindow::set_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom){

}
void MainWindow::set_mavlinkState(string state){
    ui.label_mavlinkState->setText(state.c_str());
}
void MainWindow::set_mavlinkType(string type){
    ui.label_mavlinkType->setText(type.c_str());
}

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
/*void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}*/

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/


/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace drone_monitor


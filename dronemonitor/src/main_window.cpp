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
#include "../include/dronemonitor/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dronemonitor {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
	setWindowIcon(QIcon(":/images/icon.png"));
    ui->tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::set_armed(bool isArmed){
    if(isArmed){
        ui->label_armed->setText(QString("<font color='green'>ARMED</font>"));
    }else{
        ui->label_armed->setText(QString("<font color='red'>ARMED</font>"));
    }

}

void MainWindow::set_status(int status){
    if(status == 1){
        ui->label_status->setText(QString("In Air"));
    }else if(status == 2){

    }else if(status == 3){

    }
}

void MainWindow::set_battery(double SOC){

}

void MainWindow::set_time(string time){

}

void MainWindow::set_uptime(double sec){

}

void MainWindow::set_RPY(vector<double> RPY){

}

void MainWindow::set_heading(double heading){

}

void MainWindow::set_position(gcs::GPS pos){

}
void MainWindow::set_Height(double height){

}

void MainWindow::set_velocity(double velocity){

}

void MainWindow::set_autoPilot(string text){

}

void MainWindow::set_subFlightmode(string text){

}

void MainWindow::set_mainFlightmode(string text){

}

void MainWindow::set_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom){

}

void MainWindow::set_mavlinkState(string state){

}

void MainWindow::set_mavlinkType(string type){

}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/



void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace dronemonitor


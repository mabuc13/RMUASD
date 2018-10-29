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
#include <QLocale>
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
        ui->label_status->setText(QString("Running Mission"));
    }else if(status == 2){
        ui->label_status->setText(QString("On Ground"));
    }else if(status == 3){
        ui->label_status->setText(QString("On hold"));
    }
}

void MainWindow::set_battery(double SOC){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(SOC,'f',2));
    text.push_back(QString("%"));
    ui->label_battery->setText(text);

}

void MainWindow::set_time(string time){
    ui->label_time->setText(time.c_str());
}

void MainWindow::set_uptime(double sec){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(sec,'f',2));
    text.push_back(QString("s"));
    ui->label_uptime->setText(text);
}

void MainWindow::set_RPY(vector<double> RPY){

}

void MainWindow::set_heading(double heading){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(heading,'f',2));
    //text.push_back(QString("s"));
    ui->label_heading->setText(text);
}

void MainWindow::set_position(gcs::GPS pos){

}
void MainWindow::set_Height(double height){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(height,'f',1));
    text.push_back(QString("m"));
    ui->label_height->setText(text);
}

void MainWindow::set_velocity(double velocity){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(velocity,'f',2));
    text.push_back(QString("m/s"));
    ui->label_velocity->setText(text);
}

void MainWindow::set_autoPilot(string text){
    ui->label_autoPilot->setText(text.c_str());
}

void MainWindow::set_subFlightmode(string text){
    ui->label_filghtmode_sub->setText(text.c_str());

}

void MainWindow::set_mainFlightmode(string text){
    ui->label_flightmode_main->setText(text.c_str());
}

void MainWindow::set_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom){

}

void MainWindow::set_mavlinkState(string state){
    ui->label_mavlinkState->setText(state.c_str());
}

void MainWindow::set_mavlinkType(string type){
    ui->label_mavlinkType->setText(type.c_str());
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/



void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace dronemonitor


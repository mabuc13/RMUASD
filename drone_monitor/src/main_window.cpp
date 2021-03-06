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
#include <string>

#ifndef Q_MOC_RUN
	#include <node_monitor/heartbeat.h>
	#include <node_monitor/nodeOk.h>
#endif

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

    //############### Element initialization #####################
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    ui.textEdit_terminal->setStyleSheet("background-color: black");
    ui.label_droneHandler->setWordWrap(true);

    //############### Manualy connected SLOTS and SIGNALS ###############
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(sig_armed(bool)), this, SLOT(set_armed(bool)));
    QObject::connect(&qnode, SIGNAL(sig_status(int)), this, SLOT(set_status(int)));
    QObject::connect(&qnode, SIGNAL(sig_battery(double)), this, SLOT(set_battery(double)));
    QObject::connect(&qnode, SIGNAL(sig_time(long)), this, SLOT(set_time(long)));
    QObject::connect(&qnode, SIGNAL(sig_uptime(double)), this, SLOT(set_uptime(double)));
    QObject::connect(&qnode, SIGNAL(sig_RPY(double,double,double)), this, SLOT(set_RPY(double,double,double)));
    QObject::connect(&qnode, SIGNAL(sig_heading(double)), this, SLOT(set_heading(double)));
    QObject::connect(&qnode, SIGNAL(sig_position(double,double,double)), this, SLOT(set_position(double,double,double)));
    QObject::connect(&qnode, SIGNAL(sig_nextWayPoint(double,double,double)), this, SLOT(set_nextWayPoint(double,double,double)));
    QObject::connect(&qnode, SIGNAL(sig_Height(double)), this, SLOT(set_Height(double)));
    QObject::connect(&qnode, SIGNAL(sig_velocity(double)), this, SLOT(set_velocity(double)));
    QObject::connect(&qnode, SIGNAL(sig_autoPilot(QString)), this, SLOT(set_autoPilot(QString)));
    QObject::connect(&qnode, SIGNAL(sig_subFlightmode(QString)), this, SLOT(set_subFlightmode(QString)));
    QObject::connect(&qnode, SIGNAL(sig_mainFlightmode(QString)), this, SLOT(set_mainFlightmode(QString)));
    QObject::connect(&qnode, SIGNAL(sig_otherFlightmode(bool,bool,bool,bool,bool,bool,bool)), this, SLOT(set_otherFlightmode(bool,bool,bool,bool,bool,bool,bool)));
    QObject::connect(&qnode,SIGNAL(sig_mavlinkState(QString)),this,SLOT(set_mavlinkState(QString)));
    QObject::connect(&qnode,SIGNAL(sig_mavlinkType(QString)),this,SLOT(set_mavlinkType(QString)));
    QObject::connect(&qnode,SIGNAL(sig_missionIndex(int)),this,SLOT(set_missionIndex(int)));
    QObject::connect(&qnode,SIGNAL(sig_missionLength(int)),this,SLOT(set_missionLength(int)));
    QObject::connect(&qnode,SIGNAL(sig_droneMission(int,int)),this,SLOT(set_droneMission(int,int)));
    QObject::connect(&qnode,SIGNAL(sig_ETA(int)),this,SLOT(set_ETA(int)));
    QObject::connect(&qnode,SIGNAL(sig_droneHandlerState(QString)),this,SLOT(set_droneHandlerState(QString)));
    QObject::connect(&qnode,SIGNAL(sig_telemetryStatus(int,QString)),this,SLOT(set_telemetryStatus(int,QString)));
    QObject::connect(&qnode,SIGNAL(sig_gcsJobState(int,QString)),this,SLOT(set_gcsJobState(int,QString)));
    QObject::connect(&qnode,SIGNAL(sig_nodeState(QString,int,int)),this,SLOT(set_nodeState(QString,int,int)));
    qnode.init();

    ui.gridLayout_NodeStates->addWidget(new QLabel("<b>NodeName<\b>"),0,0);
    ui.gridLayout_NodeStates->addWidget(new QLabel("<b>Response<\b>"),0,1);
    ui.gridLayout_NodeStates->addWidget(new QLabel("<b>Status<\b>"),0,2);
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/
namespace  {
QString getResponseText(int response){
    QString text;
    if(response == node_monitor::nodeOk::fine){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#00ff00;'>Fine</span></p>");
    }else if(response == node_monitor::nodeOk::late){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#ffff00;'>Late</span></p>");
    }else if(response == node_monitor::nodeOk::none_responsive){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#ff0000;'>Dead</span></p>");
    }else{
       text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#000000;'>ERROR</span></p>");
    }
    return text;
}
QString getStatusText(int status){
    QString text;
    if(status == node_monitor::heartbeat::nothing || status == node_monitor::heartbeat::info){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#00ff00;'>Fine</span></p>");
    }else if(status == node_monitor::heartbeat::warning){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#ffff00;'>Warning</span></p>");
    }else if(status == node_monitor::heartbeat::error){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#ff2000;'>Error</span></p>");
    }else if(status == node_monitor::heartbeat::critical_error){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#ff1000;'>Critical</span></p>");
    }else if(status == node_monitor::heartbeat::fatal_error){
        text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#ff0000;'>Fatal</span></p>");
    }else{
       text = QString(
            "<p><span style=' font-size:9pt; font-weight:600; color:#000000;'>Unknown</span></p>");
    }
    return text;
}
}

void MainWindow::set_nodeState(QString name, int ownStatus, int response){
    
    bool newNode = true;
    //std::cout << name.toUtf8().constData() << endl;
    for(size_t i = 0; i < this->monitoredNodes.size(); i++){
        if(this->monitoredNodes[i].name->text() == name){
            newNode = false;
            this->monitoredNodes[i].state->setText(getStatusText(ownStatus));
            this->monitoredNodes[i].response->setText(getResponseText(response));
        }
    }
    if(newNode){
        QNodeStateLabel tmp;
        tmp.name = new QLabel(name);
        tmp.response = new QLabel(getResponseText(response));
        tmp.state = new QLabel(getStatusText(ownStatus));
        ui.gridLayout_NodeStates->addWidget(tmp.name,monitoredNodes.size()+1,0);
        ui.gridLayout_NodeStates->addWidget(tmp.response,monitoredNodes.size()+1,1);
        ui.gridLayout_NodeStates->addWidget(tmp.state,monitoredNodes.size()+1,2);
        monitoredNodes.push_back(tmp);
    }

}


void MainWindow::set_armed(bool isArmed){
    if(isArmed){
        ui.label_armed->setText(QString("<p><span style=' font-size:12pt; font-weight:600; color:#00ff00;'>Armed</span></p>"));
    }else{
        ui.label_armed->setText(QString("<p><span style=' font-size:12pt; font-weight:600; color:#ff0000;'>Not Armed</span></p>"));
    }
}

void MainWindow::set_status(int status){
    if(status == 1){
        ui.label_status->setText(QString("<span style=' font-size:12pt; font-weight:600;'>Running Mission</span>"));
    }else if(status == 2){
        ui.label_status->setText(QString("<span style=' font-size:12pt; font-weight:600;'>On Ground</span>"));
        ui.label_ETA->setText(QString("N/A"));
        ui.label_missinLenght->setText(QString("N/A"));
        ui.label_missionIndex->setText(QString("N/A"));
        ui.label_nextAltitude->setText(QString("N/A"));
        ui.label_nextLatitude->setText(QString("N/A"));
        ui.label_longitude->setText(QString("N/A"));
    }else if(status == 3){
        ui.label_status->setText(QString("<span style=' font-size:12pt; font-weight:600;'>On Hold</span>"));
    }
}
void MainWindow::set_battery(double SOC){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(SOC,'f',2));
    text.push_back(QString("%"));
    ui.label_battery->setText(text);

}
void MainWindow::set_time(long GPStime){
    QString text = QString::number(GPStime);
    text.push_back(QString("s"));
    ui.label_time->setText(text);
}
void MainWindow::set_uptime(double sec){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(sec,'f',0));
    text.push_back(QString("s"));
    ui.label_uptime->setText(text);
}
void MainWindow::set_RPY(double R, double P, double Y){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(R,'f',1));
    text.push_back(", ");
    text.push_back(obj.toString(P,'f',1));
    text.push_back(", ");
    text.push_back(obj.toString(Y,'f',1));
    ui.label_RPY->setText(text);
}
void MainWindow::set_heading(double heading){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(heading,'f',2));
    //text.push_back(QString("s"));
    ui.label_heading->setText(text);
}
void MainWindow::set_position(double altitude,double longitude, double latitude){
    QString text;
    QLocale obj;

    text.push_back(obj.toString(latitude,'f',6));
    text.push_back(QString("°"));
    ui.label_latitude->setText(text);
    text.clear();

    text.push_back(obj.toString(longitude,'f',6));
    text.push_back(QString("°"));
    ui.label_longitude->setText(text);
}
void MainWindow::set_nextWayPoint(double altitude,double longitude, double latitude){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(altitude,'f',1));
    text.push_back(QString("m"));
    ui.label_nextAltitude->setText(text);
    text.clear();

    text.push_back(obj.toString(latitude,'f',6));
    text.push_back(QString("°"));
    ui.label_nextLatitude->setText(text);
    text.clear();

    text.push_back(obj.toString(longitude,'f',6));
    text.push_back(QString("°"));
    ui.label_nextLongitude->setText(text);
}
void MainWindow::set_missionIndex(int len){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(double(len),'f',0));
    ui.label_missionIndex->setText(text);
}
void MainWindow::set_missionLength(int len){
    QString text;
    QLocale obj;
    text.push_back(obj.toString(double(len),'f',0));
    ui.label_missinLenght->setText(text);
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
void MainWindow::set_autoPilot(QString text){
    ui.label_autoPilot->setText(text);
}
void MainWindow::set_subFlightmode(QString text){
    ui.label_filghtmode_sub->setText(text);

}
void MainWindow::set_mainFlightmode(QString text){
    ui.label_flightmode_main->setText(text);
}
void MainWindow::set_otherFlightmode(bool manual, bool simulation, bool stabilized, bool guided, bool autoM, bool test, bool custom){
    QString text;
    if(manual) text.push_back(QString("manual,"));
    if(simulation) text.push_back(QString("simulation,"));
    if(stabilized) text.push_back(QString("stabilized,"));
    if(guided) text.push_back(QString("guided,"));
    if(autoM) text.push_back(QString("auto,"));
    if(test) text.push_back(QString("test,"));
    if(custom) text.push_back(QString("custom,"));
    if(text.size() == 0) text.push_back(QString("none"));
    ui.label_flightmode_other->setText(text);
}
void MainWindow::set_mavlinkState(QString state){
    ui.label_mavlinkState->setText(state);
}
void MainWindow::set_mavlinkType(QString type){
    ui.label_mavlinkType->setText(type);
}

void MainWindow::set_droneHandlerState(QString text){
    ui.label_droneHandler->setText(text);

}
void MainWindow::set_ETA(int sec){
    QString text;
    QLocale obj;
    text.push_back("T-");
    text.push_back(obj.toString(double(sec),'f',0));
    text.push_back(QString("s"));
    ui.label_ETA->setText(text);
}
void MainWindow::set_droneMission(int index, int len){
    QString text;
    QLocale obj;
    text.push_back("[");
    text.push_back(obj.toString(double(index),'f',0));
    text.push_back("/");
    text.push_back(obj.toString(double(len),'f',0));
    text.push_back(QString("]"));
    ui.label_droneMission->setText(text);
}

void MainWindow::set_telemetryStatus(int severity, QString text){



    if(severity == 5){
        ui.textEdit_terminal->setTextColor(QColor(255,255,0));
    }else if(severity == 4){
        ui.textEdit_terminal->setTextColor(QColor(255,200,0));
    }else if(severity == 3){
        ui.textEdit_terminal->setTextColor(QColor(255,150,0));
    }else if(severity == 2){
        ui.textEdit_terminal->setTextColor(QColor(255,100,0));
    }else if(severity == 1){
        ui.textEdit_terminal->setTextColor(QColor(255,50,0));
    }else if(severity == 0){
        ui.textEdit_terminal->setTextColor(QColor(255,0,0));
    }else{
        ui.textEdit_terminal->setTextColor(QColor(255,255,255));
    }

    bool atBtn = false;
    if(ui.textEdit_terminal->verticalScrollBar()->value() == ui.textEdit_terminal->verticalScrollBar()->maximum())
        atBtn = true;
    ui.textEdit_terminal->insertPlainText(text);
    if(atBtn)
        ui.textEdit_terminal->verticalScrollBar()->setValue(ui.textEdit_terminal->verticalScrollBar()->maximum());
}
void MainWindow::set_gcsJobState(int state, QString text){
    QString t;
    QLocale obj;
    t.push_back("[");
    t.push_back(obj.toString(double(state),'f',0));
    t.push_back(QString("]: "));
    t.push_back(text);
    ui.label_gcsJobStatus->setText(t);
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


#include <iostream>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <map>

#include <locale>

#include <ros/ros.h>
#include <ros/package.h>

#include <gcs/GPS.h>
#include <gcs/DroneInfo.h>
#include <gcs/DronePath.h>
#include <gcs/DroneSingleValue.h>
#include <std_msgs/String.h>
#include <internet/getIp.h>
#include <gcs/pathPlan.h>
#include <gcs/getEta.h>
#include <gcs/gps2distance.h>
#include <node_monitor/heartbeat.h>

#include <DronesAndDocks.hpp>
#include <TextCsvReader.hpp>

using namespace std;

#define DEBUG true

ros::Subscriber DroneStatus_sub;//= rospy.Subscriber('/Telemetry/DroneStatus',DroneInfo, DroneStatus_handler)
ros::Publisher RouteRequest_pub;// = rospy.Publisher('/gcs/PathRequest', DronePath, queue_size=10)

ros::Subscriber WebInfo_sub;// = rospy.Subscriber('/FromInternet',String, Web_handler)
ros::Publisher WebInfo_pub;// = rospy.Publisher('/ToInternet', String, queue_size = 10)

ros::Publisher ETA_pub;
ros::Publisher JobState_pub;
ros::Publisher Heartbeat_pub;

ros::ServiceClient pathPlanClient;
ros::ServiceClient EtaClient;
ros::ServiceClient GPSdistanceClient;
ros::ServiceClient client;


ros::NodeHandle* nh;

node_monitor::heartbeat heartbeat_msg;

std::deque<job*> jobQ;
std::vector<dock*> Docks;
std::vector<drone*> Drones;
std::deque<job*> activeJobs;

ostream& operator<<(ostream& os, const gcs::GPS& pos)  
{  
    os << "Lon(" << pos.longitude << "), Lat(" << pos.latitude << "), Alt(" << pos.altitude << ")";  
    return os;  
} 

void NodeState(uint8 severity,string msg,double rate = 0){
    double curRate = heartbeat_msg.rate;
    if(rate != 0){
        heartbeat_msg.rate = rate;
    }
    heartbeat_msg.severity = severity;
    heartbeat_msg.text = msg;
    heartbeat_msg.header.stamp = ros::Time::now();
    Heartbeat_pub.publish(heartbeat_msg);
    heartbeat_msg.rate = curRate;
}

// ######### Service calls ###########
double GPSdistance(const gcs::GPS &point1, const gcs::GPS &point2){
    gcs::gps2distance srv;
    srv.request.point1 = point1;
    srv.request.point2 = point2;
    bool worked = GPSdistanceClient.call(srv);
    if (worked){
        if(DEBUG) cout << "[Ground Control]: " << "Distance calculated " << srv.response.distance << endl;
    }else{
        cout << "[Ground Control]: " << "Distance Calc failed"<< endl;
    }
    return srv.response.distance;
}

std::vector<gcs::GPS> pathPlan(gcs::GPS start,gcs::GPS end){

    gcs::pathPlan srv;
    srv.request.start = start;
    srv.request.end = end;
    bool worked;
    NodeState(node_monitor::heartbeat::nothing,"",0.2);
    if(DEBUG) cout << "######################################" << endl << "Calculate Path Plan" << endl << "From: " << start <<endl <<"To: " << end << endl << "#####################################" <<endl;
    do{
        worked = pathPlanClient.call(srv);
        if (worked){
        // cout << "[Ground Control]: " << "PathPlanDone" << endl;
        }else{
            cout << "[Ground Control]: " << "PathPlanFailed"<< endl;
            NodeState(node_monitor::heartbeat::critical_error,"PathPlanner Not Responding",0.1);
        }
    }while(!worked && ros::ok());
    NodeState(node_monitor::heartbeat::nothing,"");
    

    return srv.response.path;
}
int ETA(job* aJob){
    gcs::getEta srv;

    std::vector<gcs::GPS> path = aJob->getDrone()->getPath();
    std::deque<gcs::GPS> part(path.begin()+aJob->getDrone()->getMissionIndex(),path.end());
    part.push_front(aJob->getDrone()->getPosition());
    srv.request.path = std::vector<gcs::GPS> (part.begin(),part.end());
    srv.request.speed = aJob->getDrone()->getVelocity();

    bool worked = EtaClient.call(srv);

    // if (worked){
    //     cout << "[Ground Control]: " << "ETA calculated " << srv.response.eta << endl;
    // }else{
    //     cout << "[Ground Control]: " << "ETA failed"<< endl;
    // }
    return srv.response.eta;
}

// ######### Helper functions ###########
bool appendDockingStation(string textIn){
    CSVmsg text(textIn);
    if(text.hasValue("name") &&
       text.hasValue("longitude") &&
       text.hasValue("latitude") &&
       text.hasValue("altitude"))
    {
        Docks.push_back(
            new dock(text.getValue("name"),
                 text.getNumValue("latitude"),
                 text.getNumValue("longitude"),
                 text.getNumValue("altitude"),
                 text.getValue("lab") == "true"
            )
        );
        return true;
    }else{
        return false;
    }
}
void webMsg(dock* reciver, string msg){
    std_msgs::String msgOut;
    string m;
    m+= "name=gcs,target=";
    m+=reciver->getName();
    m+=",";
    m+=msg;
    msgOut.data=m;
    WebInfo_pub.publish(msgOut);
    //cout << "[Ground Control]: " << m << endl;
}
dock* closestLab(drone* theDrone){ // TODO
    double distance = INFINITY;
    dock* theLab;
    for(size_t i = 0; i < Docks.size(); i++){
        if(Docks[i]->isLab()){
            double d = GPSdistance(Docks[i]->getPosition(),theDrone->getPosition());
            if(d < distance){
                distance = d;
                theLab = Docks[i];
            }
        }
    }
    return theLab;
}

// ######### Message Handers ##########
void DroneStatus_Handler(gcs::DroneInfo msg){
    // ###################### Is Drone Registered #####################
    bool isANewDrone = true;
    size_t index = 0;
    for(std::size_t i = 0; i < Drones.size();i++){
        if( Drones[i]->getID() == msg.drone_id){
            isANewDrone = false;
            index = i;
            break;
        }
    }

    if(isANewDrone){ // ############## Register if new drone #####################
        Drones.push_back(new drone(msg.drone_id,msg.position));
        cout << "[Ground Control]: " << "Drone Registered: " << Drones.back()->getID() << "at : " << msg.position << endl;
    }else{ // ################### Update Drone state #############################
        Drones[index]->setPosition(msg.position);
        Drones[index]->setMissionIndex(msg.mission_index);
        Drones[index]->setVelocity(msg.ground_speed);
        if(msg.status == msg.Run){
            job* aJob = Drones[index]->getJob();
            if(aJob != NULL){
                if(aJob->getStatus() != job::ongoing){
                    aJob->setStatus(job::ongoing);
                }
            }
            else
            {
                // std::cout << "Job is null - run" << std::endl;
            }
        }else if(msg.status == msg.Land){
            job* aJob = Drones[index]->getJob();
            if(aJob != NULL){
                if(aJob->getStatus() == job::ongoing){
                    if(aJob->getGoal() != aJob->getQuestHandler()){
                        aJob->setStatus(job::done);
                    }else{
                        aJob->setStatus(job::onhold);
                        webMsg(aJob->getQuestHandler(),"request=arrived");
                    }
                }
            }
            else
            {
                // std::cout << "Job is null - run" << std::endl;
            }
        }
    }
}
void WebInfo_Handler(std_msgs::String msg_in){
    CSVmsg msg(msg_in.data);
    cout << "[Ground Control]: "<< "MSG recived: "<<msg_in.data << endl;
    if(msg.getValue("name")=="server")
        return;
    if(msg.hasValue("name")){
        string feedback = "name=gcs,target=" + msg.getValue("name");
        if(msg.hasValue("register")){ // ##############  REGISTER #############
            if(appendDockingStation(msg.getText())){
                cout << "[Ground Control]: " << "Docking Station added: " << Docks.back()->getName()<< endl;
                feedback += ",register=succes";
            }else{
                feedback += ",register=failed";
            }
        }else if(msg.hasValue("request")){  // ##########  REQUEST ##############
            if(DEBUG) cout << "[Ground Control]: " << "request recived" << endl;
            dock* goalDock = NULL;
            string dockName = msg.getValue("name");
            for(size_t i = 0; i < Docks.size(); i++){
                if(Docks[i]->getName() == dockName){
                    if(DEBUG) cout << "[Ground Control]: " << "Dock found" << endl;
                    goalDock = Docks[i];
                }
            }
            if(goalDock != NULL){
                if(DEBUG) cout << "[Ground Control]: " << "making job" << endl;
                jobQ.push_back(new job(goalDock));
                feedback+= ",request=queued";
            }else{
                feedback+= ",request=failed,error=No Dockingstation named " + msg.getValue("name");
            }
            if(DEBUG) cout << "[Ground Control]: " << "request end" << endl;
        }else if(msg.hasValue("return")){  // ##########  RETURN ###############
            job* theJob;
            for(size_t i = 0; i < activeJobs.size();i++){
                if(activeJobs[i]->getStatus() == job::onhold){
                    if(activeJobs[i]->getQuestHandler()->getName() == msg.getValue("name")){
                        dock* goalDock = NULL;
                        string dockName = msg.getValue("return");
                        if(dockName.size() <1){ // if no return addresse find nearest lab
                            goalDock = closestLab(activeJobs[i]->getDrone());
                        }else{ // Else find named lab
                            for(size_t i = 0; i < Docks.size(); i++){
                                if(Docks[i]->getName() == dockName){
                                    goalDock = Docks[i];
                                    break;
                                }
                            }
                        }
                        if(goalDock == NULL){
                            goalDock=closestLab(activeJobs[i]->getDrone());
                            if(goalDock == NULL){
                                feedback+= ",return=failed,error=Lab not found";
                                break;
                            }
                        }

                        activeJobs[i]->setGoal(goalDock);
                        activeJobs[i]->setStatus(job::wait4pathplan);
                        feedback+=",return=succes,request=waiting4pathplan";
                        break;
                    }
                }
            }
        }
        std_msgs::String msg_out;
        msg_out.data = feedback;
        WebInfo_pub.publish(msg_out);
    }
}



void initialize(void){
    nh = new ros::NodeHandle();
    DroneStatus_sub = nh->subscribe("/drone_handler/DroneInfo",100,DroneStatus_Handler);
    RouteRequest_pub = nh->advertise<gcs::DronePath>("/gcs/forwardPath",100);
    ETA_pub = nh->advertise<gcs::DroneSingleValue>("/gcs/ETA",100);
    WebInfo_sub = nh->subscribe("/internet/FromInternet",100,WebInfo_Handler);
    WebInfo_pub = nh->advertise<std_msgs::String>("/internet/ToInternet",100);
    JobState_pub = nh->advertise<gcs::DroneSingleValue>("/gcs/JobState",100);
    Heartbeat_pub = nh->advertise<node_monitor::heartbeat>("/node_monitor/input/Heartbeat",100);

    ifstream myFile(ros::package::getPath("gcs")+"/scripts/Settings/DockingStationsList.txt");
    if(myFile.is_open()){
        cout << endl << endl <<endl;
        cout << "[Ground Control]: " << "File opened: " <<  ros::package::getPath("gcs")<<"/scripts/Settings/DockingStationsList.txt" << endl;
        string line;
        while(getline(myFile,line)){
            //cout << "[Ground Control]: " << line << endl;
            if(appendDockingStation(line)){
                dock theDock = *(Docks.back());
                cout << "[Ground Control]: " << "Docking Station added: " << theDock.getName()<< endl;
            }else{
                cout << "[Ground Control]: " << "Not a Docking Station" << endl;
            }
        }
    }


    pathPlanClient = nh->serviceClient<gcs::pathPlan>("/pathplan/getPlan");
    client = nh->serviceClient<internet::getIp>("/Internet/getIp");
    EtaClient = nh->serviceClient<gcs::getEta>("/pathplan/getEta");
    GPSdistanceClient = nh->serviceClient<gcs::gps2distance>("pathplan/GPS2GPSdist");

    sleep(2);
    /*
    internet::getIp srv;
    srv.request.username = "waarbubble@gmail.com";
    cout << "[Ground Control]: webServer Username: " << srv.request.username <<endl;
    cout << "[Ground Control]: webServer Password: ";
    string password;
    getline(cin, password);
    if(password.size()>2){
        srv.request.password = password;
        bool worked = false;
        const long maxTries = 3;
        long tries = 0;
        cout << "[Ground Control]: " << "Getting Server IP" << endl;
        while(!worked && tries < maxTries){
            tries++;
            worked = client.call(srv);
            if (worked){
                cout << "[Ground Control]: " << "IP: " << srv.response.ip << endl;
                cout << "[Ground Control]: " << "Port: " << srv.response.port << endl;
                string msg = "name=gcs,server="+srv.response.ip+",port="+srv.response.port;
                std_msgs::String msgOut;
                msgOut.data=msg;
                WebInfo_pub.publish(msgOut);
            }else{
                cout << "[Ground Control]: webServer Password: ";
                password.clear();
                getline(cin, password);
            }
        }
        if(tries == maxTries){
            ROS_ERROR("Could not obtain IP");
        }
    }else{
        cout << "[Ground Control]: ignoring IP service" << endl;
    }
    */
}


int main(int argc, char** argv){
    ros::init(argc,argv,"gcs");
    initialize();
    unsigned long spins = 0;

    int rate = 10;
    ros::Rate r(rate);
    heartbeat_msg.rate = rate;
    heartbeat_msg.severity = node_monitor::heartbeat::nothing;
    heartbeat_msg.header.frame_id ="gcs";
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        heartbeat_msg.header.stamp = ros::Time::now();
        Heartbeat_pub.publish(heartbeat_msg);

        // ############ Find Available drones for queued Jobs ############
        if(jobQ.size() != 0){
            for(size_t i = 0; i < Drones.size();i++){
                if(Drones[i]->isAvailable()){
                    activeJobs.push_back(jobQ.front());
                    jobQ.pop_front();
                    Drones[i]->setJob(activeJobs.back());
                    Drones[i]->setAvailable(false);
                    Drones[i]->getJob()->setStatus(job::wait4pathplan);
                    webMsg(Drones[i]->getJob()->getQuestHandler(),"request=pathplaning");
                }
            }
        }

        // ############### Do path planing for all jobs waiting for new Path plan ################
        for(size_t i = 0; i < activeJobs.size();i++){
            if(activeJobs[i]->getStatus()==job::wait4pathplan){
                gcs::GPS start = activeJobs[i]->getDrone()->getPosition();
                gcs::GPS end = activeJobs[i]->getGoal()->getPosition();
                // always start and stop above docking stations
                start.altitude = 32;
                end.altitude = 32;
                std::vector<gcs::GPS> path = pathPlan(start, end);

                gcs::DronePath msg;
                msg.Path = path;
                msg.DroneID = activeJobs[i]->getDrone()->getID();

                RouteRequest_pub.publish(msg);
                activeJobs[i]->setStatus(job::ready4takeOff);
                activeJobs[i]->getDrone()->setPath(path);

                webMsg(activeJobs[i]->getQuestHandler(),"request=ready4takeoff");

            }
        }

        // ############## Send out INFO on Drone ETA ######################
        if(spins >= rate){
            spins =0;
            for(size_t i = 0; i < activeJobs.size(); i++){
                if(activeJobs[i]->getStatus()==job::ongoing){
                     double eta = ETA(activeJobs[i]);
                     webMsg(activeJobs[i]->getQuestHandler(),
                            "request=ongoing,ETA="+
                            std::to_string(eta)
                            );
                     gcs::DroneSingleValue msg;
                     msg.value = eta;
                     msg.drone_id = activeJobs[i]->getDrone()->getID();
                     ETA_pub.publish(msg);
                }
            }
        }

        // #################### Job state pub ########################
        for(size_t i = 0 ; i < Drones.size(); i++){
            gcs::DroneSingleValue msg;
            msg.drone_id = Drones[i]->getID();
            if(Drones[i]->getJob() != NULL){
                msg.value = Drones[i]->getJob()->getStatus();
                if(msg.value == job::queued){
                    msg.text = "Queued";
                }else if(msg.value == job::ongoing){
                    msg.text = "Ongoing";
                }else if(msg.value == job::onhold){
                    msg.text = "Onhold";
                }else if(msg.value == job::wait4pathplan){
                    msg.text = "Waiting for pathplan";
                }else if(msg.value == job::ready4takeOff){
                    msg.text = "Ready for takeoff";
                }else if(msg.value == job::done){
                    msg.text = "Done";
                }
            }else{
                msg.value = job::noMission;
                msg.text = "No Job assigned";
            }
            JobState_pub.publish(msg);
        }

        // ############### Delete Finnished Jobs ######################
        for(size_t i = 0 ; i< activeJobs.size();i++){
            if(activeJobs[i]->getStatus() == job::done){
                delete activeJobs[i];
                activeJobs.erase(activeJobs.begin()+i);
                i--;
            }
        }

        spins++;
    }

    delete nh;
}

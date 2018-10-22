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
#include <gcs/DroneState.h>
#include <std_msgs/String.h>
#include <internet/getIp.h>

#include <DronesAndDocks.hpp>
#include <TextCsvReader.hpp>

using namespace std;
#ifndef GCS_MESSAGE_GPS_H
    namespace gcs{
        struct GPS{
            double longitude;
            double latitude;
            double altitude;
        };

        struct DroneInfo{
            GPS position;
            GPS next_goal;
            float velocity[3];
            float heading;
            float battery_SOC;

            size_t drone_id;
            size_t GPS_timestamp;
            uint8 status;

            uint8 Run = 1;
            uint8 Stop = 2;
            uint8 Land = 3;
            uint8 Wait = 4;
        };

        struct DronePath{
            std::vector<GPS> Path;
            size_t DroneID;
        };
    }
#endif


<<<<<<< HEAD
ros::Subscriber DroneStatus_sub;//= rospy.Subscriber('/Telemetry/DroneStatus',DroneInfo, DroneStatus_handler)
ros::Publisher RouteRequest_pub;// = rospy.Publisher('/gcs/PathRequest', DronePath, queue_size=10)

ros::Subscriber WebInfo_sub;// = rospy.Subscriber('/FromInternet',String, Web_handler)
ros::Publisher WebInfo_pub;// = rospy.Publisher('/ToInternet', String, queue_size = 10)
=======
ros::Subscriber DroneStatus_sub;//= rospy.Subscriber('/Telemetry/DroneStatus',DroneInfo, DroneStatus_handler) // NOLINT
ros::Publisher RouteRequest_pub;// = rospy.Publisher('/gcs/PathRequest', DronePath, queue_size=10) // NOLINT
//ros::Publisher DroneState_pub;// = rospy.Publisher('/gcs/StateRequest', DroneState, queue_size=10)
ros::Subscriber WebInfo_sub;// = rospy.Subscriber('/FromInternet',String, Web_handler) // NOLINT
ros::Publisher WebInfo_pub;// = rospy.Publisher('/ToInternet', String, queue_size = 10) // NOLINT
>>>>>>> e2a0f0d45165205b593976d9936a2f4cd9fc2635
ros::NodeHandle* nh;

std::deque<job*> jobQ;
std::vector<dock*> Docks;
std::vector<drone*> Drones;
std::deque<job*> activeJobs;




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

dock* closestLab(drone* theDrone){ // TODO
    for(size_t i = 0; i < Docks.size(); i++){
        if(Docks[i]->isLab()){
            return Docks[i];
        }
    }
    return NULL;
}
std::vector<gcs::GPS> pathPlan(gcs::GPS start,gcs::GPS end){
    std::vector<gcs::GPS> ret = {start,end};
    return ret;
}
int ETA(job* aJob){
    return 3600;
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
    cout << m << endl;
}


void DroneStatus_Hangler(gcs::DroneInfo msg){

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
        cout << "Drone Registered: " << Drones.back()->getID() << endl;
    }else{ // ################### Update Drone state #############################
        Drones[index]->setPosition(msg.position);
        if(msg.status == msg.Run){
            job* aJob = Drones[index]->getJob();
            if(aJob->getStatus() != job::ongoing){
                aJob->setStatus(job::ongoing);
            }
        }else if(msg.status == msg.Land){
            job* aJob = Drones[index]->getJob();
            if(aJob->getStatus() == job::ongoing){
                if(aJob->getGoal() == aJob->getQuestHandler()){
                    aJob->setStatus(job::done);
                }else{
                    aJob->setStatus(job::onhold);
                    webMsg(aJob->getQuestHandler(),"request=arrived");
                }
            }
        }
    }
}

void WebInfo_Handler(std_msgs::String msg_in){
    CSVmsg msg(msg_in.data);
    cout << msg_in.data << endl;
    if(msg.hasValue("name") ){
        string feedback = "name=gcs,target=" + msg.getValue("name");
        if(msg.hasValue("register")){ // ##############  REGISTER #############
            if(appendDockingStation(msg.getText())){
                cout << "Docking Station added: " << Docks.back()->getName()<< endl;
                feedback += ",register=succes";
            }else{
                feedback += ",register=failed";
            }
        }else if(msg.hasValue("request")){  // ##########  REQUEST ##############
            cout << "request recived" << endl;
            dock* goalDock;
            string dockName = msg.getValue("name");
            for(size_t i = 0; i < Docks.size(); i++){
                if(Docks[i]->getName() == dockName){
                    cout << "Dock found" << endl;
                    goalDock = Docks[i];
                }
            }
            if(goalDock != NULL){
                cout << "making job" << endl;
                jobQ.push_back(new job(goalDock));
                feedback+= ",request=queued";
            }else{
                feedback+= ",request=No Dockingstation named " + msg.getValue("name");
            }
            cout << "request end" << endl;
        }else if(msg.hasValue("return")){  // ##########  RETURN ###############
            job* theJob;
            for(size_t i = 0; i < activeJobs.size();i++){
                if(activeJobs[i]->getStatus() == job::onhold){
                    if(activeJobs[i]->getQuestHandler()->getName() == msg.getValue("name")){

                        dock* goalDock;
                        string dockName = msg.getValue("return");
                        for(size_t i = 0; i < Docks.size(); i++){
                            if(Docks[i]->getName() == dockName){
                                goalDock = Docks[i];
                                break;
                            }
                        }

                        if(goalDock == NULL){
                            goalDock=closestLab(activeJobs[i]->getDrone());
                            if(goalDock == NULL){
                                feedback+= ",return=Lab not found";
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
    DroneStatus_sub = nh->subscribe("/telemetry/DroneStatus",100,DroneStatus_Hangler);
    RouteRequest_pub = nh->advertise<gcs::DronePath>("/gcs/PathRequest",100);
    WebInfo_sub = nh->subscribe("/FromInternet",100,WebInfo_Handler);
    WebInfo_pub = nh->advertise<std_msgs::String>("/ToInternet",100);

    ifstream myFile(ros::package::getPath("gcs")+"/scripts/Settings/DockingStationsList.txt");
    if(myFile.is_open()){
        cout << endl << endl <<endl;
        cout << "File opened: " <<  ros::package::getPath("gcs")<<"/scripts/Settings/DockingStationsList.txt" << endl;
        string line;
        while(getline(myFile,line)){
            //cout << line << endl;
            if(appendDockingStation(line)){
                dock theDock = *(Docks.back());
                cout << "Docking Station added: " << theDock.getName()<< endl;
            }else{
                cout << "Not a Docking Station" << endl;
            }
        }
    }
<<<<<<< HEAD

    ros::ServiceClient client = nh->serviceClient<internet::getIp>("getIp");
    internet::getIp srv;
    srv.request.username = "waarbubble@gmail.com";
    srv.request.password = "RMUASDProject";
    bool worked = false;
    const long maxTries = 10000;
    long tries = 0;
    cout << "Getting Server IP" << endl;
    while(!worked && tries < maxTries){
        tries++;
        worked = client.call(srv);
        if (worked){
            cout << "IP: " << srv.response.ip << endl;
            cout << "Port: " << srv.response.port << endl;
            string msg = "name=gcs,server="+srv.response.ip+",port="+srv.response.port;
            std_msgs::String msgOut;
            msgOut.data=msg;
            WebInfo_pub.publish(msgOut);

        }
    }

    if(tries == maxTries){
        ROS_ERROR("Could not obtain IP");
    }





=======
>>>>>>> e2a0f0d45165205b593976d9936a2f4cd9fc2635
}


int main(int argc, char** argv){
    ros::init(argc,argv,"gcs");
    initialize();
    unsigned long spins;

    while(ros::ok()){
        ros::spinOnce();

        // ############ Find Available drones for queued Jobs ############
        if(jobQ.size() != 0){
            for(size_t i = 0; i < Drones.size();i++){
                if(Drones[i]->isAvailable()){
                    cout << "Error here " << endl;
                    activeJobs.push_back(jobQ.front());
                    cout << "Error end" << endl;
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
                std::vector<gcs::GPS> path =
                        pathPlan(activeJobs[i]->getDrone()->getPosition(),
                                       activeJobs[i]->getGoal()->getPosition()
                                 );

                gcs::DronePath msg;
                msg.Path = path;
                msg.DroneID = activeJobs[i]->getDrone()->getID();

                RouteRequest_pub.publish(msg);
                activeJobs[i]->setStatus(job::ready4takeOff);
                activeJobs[i]->getDrone()->setPath(path);

                webMsg(activeJobs[i]->getQuestHandler(),"request=ready for takeoff");

            }
        }

        // ############## Send out INFO on Drone ETA ######################
        if(spins == 10000){
            spins =0;
            for(size_t i = 0; i < activeJobs.size(); i++){
                if(activeJobs[i]->getStatus()==job::ongoing){
                    webMsg(activeJobs[i]->getGoal(),
                           "request=ongoing,ETA="+
                           std::to_string(ETA(activeJobs[i]))
                           );
                }
            }
        }
        spins++;
    }

    delete nh;
}

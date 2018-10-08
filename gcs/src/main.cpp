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

#include <DronesAndDocks.hpp>

using namespace std;

ros::Subscriber DroneStatus_sub;//= rospy.Subscriber('/Telemetry/DroneStatus',DroneInfo, DroneStatus_handler)
ros::Publisher RouteRequest_pub;// = rospy.Publisher('/gcs/PathRequest', DronePath, queue_size=10)
//ros::Publisher DroneState_pub;// = rospy.Publisher('/gcs/StateRequest', DroneState, queue_size=10)
ros::Subscriber WebInfo_sub;// = rospy.Subscriber('/FromInternet',String, Web_handler)
ros::Publisher WebInfo_pub;// = rospy.Publisher('/ToInternet', String, queue_size = 10)
ros::NodeHandle* nh;

std::deque<job> jobQ;
std::vector<dock> Docks;
std::vector<drone> Drones;
std::deque<job*> activeJobs;




class CSVmsg{
public:
    CSVmsg(string text):CSVmsg(std::stringstream(text)){
    }
    CSVmsg(std::stringstream ss){
        text = ss.str();
        string comma;
        while(getline(ss,comma,',')){

            std::stringstream commaStream(comma);
            string name;
            getline(commaStream,name,'=');
            string value;
            getline(commaStream,value,'=');
            //cout << comma << " # " << name << " == " << value << endl;
            lowerCase(name);
            lowerCase(value);
            this->msg[name.c_str()] = value.c_str();
        }
    }
    string getText(void){
        return text;
    }
    bool hasValue(string name){
        try{
            lowerCase(name);
            this->msg.at(name);
            return true;
        }catch (const std::out_of_range& oor) {
            return false;
        }
    }
    string getValue(string name){
        try{
            lowerCase(name);
            return this->msg.at(name);
        }catch (const std::out_of_range& oor) {
            return "NULL";
        }
    }
    double getNumValue(string name){
        string value = getValue(name);
        //value.erase(remove_if(value.begin(), value.end(), isspace), value.end());
        try{
            //cout << "Double " << std::stod(value) << std::endl;
            return std::stod(value);
        }catch(std::invalid_argument e){

            cout << "Invalid argument: " << e.what() << endl;
            cout << "For value_" << value << "_"<<endl;
            return INFINITY;
        }


    }

private:
    void lowerCase(string & str){
        std::locale loc;
        for (std::string::size_type i=0; i<str.length(); ++i)
            str[i] = std::tolower(str[i],loc);
    }
    string text;
    std::map<string,string> msg;
    std::vector<double> test;
};

bool appendDockingStation(string textIn){
    CSVmsg text(textIn);
    if(text.hasValue("name") &&
       text.hasValue("longitude") &&
       text.hasValue("latitude") &&
       text.hasValue("altitude"))
    {
        Docks.push_back(
            dock(text.getValue("name"),
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
        if(Docks[i].isLab()){
            return &Docks[i];
        }
    }
    return NULL;
}

void DroneStatus_Hangler(gcs::DroneInfo msg){
    bool isANewDrone = true;
    size_t index = 0;
    for(std::size_t i = 0; i < Drones.size();i++){
        if( Drones[i].getID() == msg.drone_id){
            isANewDrone = false;
            index = i;
            break;
        }
    }
    if(isANewDrone){
        Drones.push_back(drone(msg.drone_id,msg.position));
        cout << "Drone Registered: " << Drones.back().getID() << endl;
    }else{
        Drones[index].setPosition(msg.position);
        if(msg.status == msg.Run){
            job* aJob = Drones[index].getJob();
            if(aJob->getStatus() != job::ongoing){
                aJob->setStatus(job::ongoing);
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
                cout << "Docking Station added: " << Docks.back().getName()<< endl;
                feedback += ",register=succes";
            }else{
                feedback += ",register=failed";
            }
        }else if(msg.hasValue("request")){  // ##########  REQUEST ##############
            dock* goalDock;
            string dockName = msg.getValue("name");
            for(size_t i = 0; i < Docks.size(); i++){
                if(Docks[i].getName() == dockName){
                    goalDock = &Docks[i];
                }
            }
            if(goalDock != NULL){
                jobQ.push_back(job(goalDock));
                feedback+= ",request=queued";
            }else{
                feedback+= ",request=No Dockingstation named " + msg.getValue("name");
            }
        }else if(msg.hasValue("return")){  // ##########  RETURN ###############
            job* theJob;
            for(size_t i = 0; i < activeJobs.size();i++){
                if(activeJobs[i]->getStatus() == job::onhold){
                    if(activeJobs[i]->getQuestHandler()->getName() == msg.getValue("name")){

                        dock* goalDock;
                        string dockName = msg.getValue("return");
                        for(size_t i = 0; i < Docks.size(); i++){
                            if(Docks[i].getName() == dockName){
                                goalDock = &Docks[i];
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
    DroneStatus_sub = nh->subscribe("/Telemetry/DroneStatus",100,DroneStatus_Hangler);
    RouteRequest_pub = nh->advertise<gcs::DronePath>("/gcs/PathRequest",100);
    WebInfo_sub = nh->subscribe("/FromInternet",100,WebInfo_Handler);
    WebInfo_pub = nh->advertise<std_msgs::String>("/ToInternet",100);

    ifstream myFile(ros::package::getPath("gcs")+"/scripts/Settings/DockingStationsList.txt");
    if(myFile.is_open()){
        cout << "File opened: " <<  ros::package::getPath("gcs")<<"/scripts/Settings/DockingStationsList.txt" << endl;
        string line;
        while(getline(myFile,line)){
            //cout << line << endl;
            if(appendDockingStation(line)){
                dock theDock = Docks.back();
                cout << "Docking Station added: " << theDock.getName()<< endl;
            }else{
                cout << "Not a Docking Station" << endl;
            }
        }
    }




}


int main(int argc, char** argv){
    ros::init(argc,argv,"gcs");
    initialize();


    while(ros::ok()){
        ros::spinOnce();
    }

    delete nh;
}

// INCLUDE STD
#include <iostream>
#include <deque>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <math.h>
#include <locale>
#include <ctime>

// INCLUDE ROS
#include <ros/ros.h>
#include <ros/package.h>

// INCLUDE MESSAGES AND SERVICES
#include <gcs/GPS.h>
#include <gcs/DroneInfo.h>
#include <gcs/DronePath.h>
#include <gcs/DroneSingleValue.h>
#include <std_msgs/String.h>
#include <internet/getIp.h>
#include <gcs/pathPlan.h>
#include <gcs/getEta.h>
#include <gcs/gps2distance.h>
#include <gcs/inCollision.h>
#include <gcs/safeTakeOff.h>
#include <gcs/moveTo.h>
#include <node_monitor/heartbeat.h>
#include <node_monitor/nodeOkList.h>
#include <node_monitor/nodeOk.h>

#include <drone_decon/RedirectDrone.h>
#include <drone_decon/RegisterDrone.h>
#include <drone_decon/GPS.h>

// INCLUDE OWN FILES
#include <DronesAndDocks.hpp>
#include <TextCsvReader.hpp>


using namespace std;

#define DEBUG true
#define DO_PREFLIGHT_CHECK true


// ############################ Clients, Subscribers and Publishers #######################
ros::Publisher RouteRequest_pub;
ros::Publisher Reposition_pub;
ros::Publisher WebInfo_pub;
ros::Publisher ETA_pub;
ros::Publisher JobState_pub;
ros::Publisher Heartbeat_pub;
ros::Publisher Transport_pub;
ros::Publisher RegisterDrone_pub;

ros::Subscriber DroneStatus_sub;
ros::Subscriber Collision_sub;
ros::Subscriber UTMDrone_sub;
ros::Subscriber WebInfo_sub;
ros::Subscriber nodeMonitor_sub;
ros::Subscriber drone_decon_sub;

ros::ServiceClient pathPlanClient;
ros::ServiceClient EtaClient;
ros::ServiceClient GPSdistanceClient;
ros::ServiceClient client;
ros::ServiceClient safeTakeOffClient;


// ############################# Global Variables ######################################
ros::NodeHandle* nh;

node_monitor::heartbeat heartbeat_msg;

std::deque<job*> jobQ;
std::vector<dock*> Docks;
std::vector<drone*> Drones;
std::deque<job*> activeJobs;
std::map<ID_t,ID_t> Own2UtmId;
std::map<ID_t,ID_t> Utm2OwnId;

node_monitor::nodeOk utm_parser;


// ################################ Misulanius ###########################################
ostream& operator<<(ostream& os, const gcs::GPS& pos)  
{  
    std::cout << std::fixed;
    std::cout << std::setprecision(6);
    os << "GPS(" << pos.latitude << ", " << pos.longitude << "), Alt(" << pos.altitude << ")";  
    return os;  
}
gcs::GPS& operator<< (gcs::GPS& pos, const drone_decon::GPS& newPos){
    pos.altitude = newPos.altitude;
    pos.longitude = newPos.longitude;
    pos.latitude = newPos.latitude;
    return pos;
}

struct is_safe_for_takeoff{
    bool takeoff_is_safe;
    uint time_til_safe_take_off;
};

// ############################ Helper functions ##################################
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
double GPSdistance(const gcs::GPS &point1, const gcs::GPS &point2);
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
double diff(double num1, double num2){
    return std::abs(num1-num2);
}
void moveDroneTo(drone *theDrone, gcs::GPS pos, long waitTill = -1){
    job* theJob =  theDrone->getJob();
    if(theJob != NULL){
        cout << "[Ground Control]: Asking for reposition" << endl;
        gcs::moveTo cmd;
        cmd.drone_id = theDrone->getID();
        cmd.position = pos;
        Reposition_pub.publish(cmd);
        if(waitTill == -1){
            theJob->setStatus(job::rePathPlan);
        }else{
            theJob->setWaitInAirTo(waitTill);
            theJob->setStatus(job::waitInAir);
            theJob->setNextStatus(job::resumeFlight);
        }
        
    }else{
        ROS_ERROR("[Ground Control]: can't reposition a drone without a job");
    }
}

void uploadFlightPlan(drone* theDrone,bool loiterAtEnd = false){
    gcs::DronePath msg;
    
    msg.DroneID = theDrone->getID();
    msg.loiterAtEnd = loiterAtEnd;
    if(DEBUG){
        while(theDrone->getPath().size()> 9){
            theDrone->getPath().erase(theDrone->getPath().end()-2);
        }
    }
    if(DEBUG) cout << "[Ground Control]: Posting PathPlan - length:" << theDrone->getPath().size() << endl;
    msg.Path = theDrone->getPath();
    RouteRequest_pub.publish(msg);
}

// ############################ Service calls ##################################
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
is_safe_for_takeoff safeTakeOff(drone* my_drone){
    gcs::safeTakeOff srv;
    srv.request.drone_id = my_drone->getID();
    srv.request.takeoff_position = my_drone->getPosition();
    bool worked = safeTakeOffClient.call(srv);

    is_safe_for_takeoff response;
    if (worked){
        if(DEBUG) cout << "[Ground Control]: " << "SafeTake off is " << int(srv.response.is_save_to_take_off) << endl;
        response.takeoff_is_safe = srv.response.is_save_to_take_off;
        response.time_til_safe_take_off = srv.response.time_to_clear;
    }else{
        cout << "[Ground Control]: " << "SafeTakeOffcheack failed"<< endl;
        NodeState(node_monitor::heartbeat::critical_error,"SafeTakeOffCheack Failed");
        response.takeoff_is_safe = false;
        response.time_til_safe_take_off = 0;
    }
    return response;
}
int ETA(job* aJob){
    gcs::getEta srv;

    std::vector<gcs::GPS> path = aJob->getDrone()->getPath();
    std::deque<gcs::GPS> part(path.begin()+aJob->getDrone()->getMissionIndex(),path.end());
    part.push_front(aJob->getDrone()->getPosition());
    srv.request.path = std::vector<gcs::GPS> (part.begin(),part.end());
    srv.request.speed = aJob->getDrone()->getVelocity();

    bool worked = EtaClient.call(srv);

    if (worked){
        //cout << "[Ground Control]: " << "ETA calculated " << srv.response.eta << endl;
    }else{
        cout << "[Ground Control]: " << "ETA failed"<< endl;
    }
    return srv.response.eta;
}
std::vector<gcs::GPS> pathPlan(gcs::GPS start,gcs::GPS end,drone* theDrone){

    gcs::pathPlan srv;
    srv.request.start = start;
    srv.request.end = end;
    srv.request.drone_id = theDrone->getID();
    
    if(theDrone->getJob() != NULL){
        if(theDrone->getJob()->getStatus() == job::wait4pathplan){
            srv.request.useDNFZ = false;
            srv.request.velocity = theDrone->getVelocitySetPoint(); // TODO get the set value from somewhere
            srv.request.startTime = std::time(nullptr)+15; //TODO is 15 the right wait amount?
        }else if(theDrone->getJob()->getStatus() == job::rePathPlan){
            srv.request.useDNFZ = true;
            srv.request.velocity = theDrone->getVelocitySetPoint();
            srv.request.startTime = std::time(nullptr)+15; // TODO calculate when drone is gonna be at first waypoint
        }else{
            ROS_ERROR("[Ground Control]: not a valid state for a pathplan");
        }
    }
    bool worked;
    NodeState(node_monitor::heartbeat::nothing,"",0.2);
    GPSdistance(start,end);
    if(DEBUG) cout << "######################################" << endl << "Calculate Path Plan" << endl << "From: " << start <<endl <<"To: " << end << endl << "#####################################" <<endl;
  
    worked = pathPlanClient.call(srv);
    if (worked){
    // cout << "[Ground Control]: " << "PathPlanDone" << endl;
    }else{
        ROS_ERROR("[Ground Control]: PathPlanFailed no contact");
        NodeState(node_monitor::heartbeat::critical_error,"PathPlanner Not Responding");
    }
    
    if(srv.response.path.size() < 2 || !worked){
        ROS_ERROR("[Ground Control]: Unacceptable Path");
        throw string("Unacceptable Path");
    }
    return srv.response.path;
}




// ############################### Message Handers ###################################
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
        
        //TODO automatic registering of drone ID
        Own2UtmId[1]=3012;
        Utm2OwnId[3012]=1;
        drone_decon::RegisterDrone reg;
        reg.drone_id = Own2UtmId[msg.drone_id];
        RegisterDrone_pub.publish(reg);
        Drones.back()->getGroundHeight() = msg.absolute_alt-msg.relative_alt;
        Drones.back()->getFlightHeight() = 32; 

        cout << "[Ground Control]: " << "Drone Registered: " << Drones.back()->getID() << "at : " << msg.position << endl;
    }else{ // ################### Update Drone state #############################
        drone_decon::RegisterDrone reg;
        reg.drone_id = Own2UtmId[msg.drone_id];
        RegisterDrone_pub.publish(reg);

        Drones[index]->setPosition(msg.position);
        if(msg.status != msg.holding) Drones[index]->setMissionIndex(msg.mission_index);
        Drones[index]->setVelocity(msg.ground_speed);
        Drones[index]->setVelocitySetPoint(msg.ground_speed_setpoint);
        Drones[index]->setStatus(msg.status);
        if(msg.status == msg.Run){
            job* aJob = Drones[index]->getJob();
            if(aJob != NULL){
                if(aJob->getStatus() == job::ready4takeOff ||
                    aJob->getStatus() == job::ready4flightContinuation)
                {
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
        }else if(msg.status == msg.holding){
            job* aJob = Drones[index]->getJob();
            if(aJob != NULL){
                if(aJob->getStatus() == job::ongoing){
                    aJob->setStatus(job::waitInAir);
                }
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
void Collision_Handler(gcs::inCollision msg){
    //TODO handle landingzone is noflight zone
    cout << "[Ground Control]: DNFZ detected" << endl;
    NodeState(node_monitor::heartbeat::info,"DNFZ detected");
    for(size_t i = 0; i < activeJobs.size(); i++){
        drone *aDrone = activeJobs[i]->getDrone();
        if(!activeJobs[i]->getDNFZinjection().stillValid&&
            (activeJobs[i]->getDNFZinjection().dnfz_id != msg.dnfz_id ||
            long(std::time(nullptr))-activeJobs[i]->getDNFZinjection().time > 30))
        {
            cout << "[Ground Control]: DNFZ is valid : " << activeJobs[i]->getDNFZinjection().stillValid << endl;
            cout << "[Ground Control]: DNFZ ID       : " << activeJobs[i]->getDNFZinjection().dnfz_id << " and " << msg.dnfz_id << endl;
            cout << "[Ground Control]: DNFZ time diff: " <<  long(std::time(nullptr))-activeJobs[i]->getDNFZinjection().time << endl;
            if(aDrone->getID() == msg.drone_id){
                if(msg.zone_type == gcs::inCollision::normal_zone){
                    activeJobs[i]->DNFZinjection(msg);
                    activeJobs[i]->getDNFZinjection().to = activeJobs[i]->getGoal()->getPosition();
                    activeJobs[i]->setStatus(job::rePathPlan);
                    activeJobs[i]->saveOldPlan();

                    vector<gcs::GPS> path = aDrone->getPath();
                    vector<gcs::GPS> newPath(path.begin()+aDrone->getMissionIndex(),path.begin()+msg.plan_index1);
                    newPath.push_back(msg.start);
                    aDrone->setPath(newPath);
                    uploadFlightPlan(aDrone,true);

                }else if(msg.zone_type == gcs::inCollision::inside_zone){
                    activeJobs[i]->DNFZinjection(msg);
                    activeJobs[i]->setStatus(job::rePathPlan);
                    activeJobs[i]->saveOldPlan();
                    
                    moveDroneTo(activeJobs[i]->getDrone(),msg.start);

                }else if(msg.zone_type == gcs::inCollision::landing_zone){
                    ROS_ERROR("[Ground Control]: can't handle obstructed ladingzone yet");
                }else{
                    ROS_ERROR("[Ground Control]: Error unrecognized dnfz type");
                }
            }else{
                cout << "Wrong drone ID" << endl;
            }
        }else{
            NodeState(node_monitor::heartbeat::info,"DNFZ ignored");
            cout << "[Ground Control]: DNFZ ignored" << endl;
            cout << "[Ground Control]: DNFZ is valid : " << activeJobs[i]->getDNFZinjection().stillValid << endl;
            cout << "[Ground Control]: DNFZ ID       : " << activeJobs[i]->getDNFZinjection().dnfz_id << " and " << msg.dnfz_id << endl;
            cout << "[Ground Control]: DNFZ time diff: " <<  long(std::time(nullptr))-activeJobs[i]->getDNFZinjection().time << endl;
        }
    }
}
void nodeMonitor_Handler(node_monitor::nodeOkList msg){
    for(size_t i = 0; i < msg.Nodes.size(); i++){
        if(msg.Nodes[i].name == "utm_parser"){
            utm_parser = msg.Nodes[i];
        }
    }
}
void deconflict_Handler(drone_decon::RedirectDrone msg){
    for(size_t i = 0; i < Drones.size(); i++){
        if(Utm2OwnId[msg.drone_id]==Drones[i]->getID()){
            gcs::GPS pos;
            pos << msg.position;
            pos.altitude-= Drones[i]->getGroundHeight();
            if(pos.altitude < 5) pos.altitude = 3;
            bool changed = false;
            if(msg.insertBeforeNextWayPoint){
                if(GPSdistance(pos,Drones[i]->getPath()[Drones[i]->getMissionIndex()])>5 &&
                    std::abs(pos.altitude-Drones[i]->getPath()[Drones[i]->getMissionIndex()].altitude)>3)
                {
                    changed = true;
                    Drones[i]->getPath().insert(Drones[i]->getPath().begin()+Drones[i]->getMissionIndex(),pos);
                    for(size_t i2 = Drones[i]->getMissionIndex()+1; i2< Drones[i]->getPath().size(); i2++){
                        Drones[i]->getPath()[i2].altitude=pos.altitude;
                    }
                }
            }else{
                if(GPSdistance(pos,Drones[i]->getPath()[Drones[i]->getMissionIndex()+1])>5 &&
                    std::abs(pos.altitude-Drones[i]->getPath()[Drones[i]->getMissionIndex()+1].altitude)>3)
                {   
                    changed = true;
                    Drones[i]->getPath().insert(Drones[i]->getPath().begin()+Drones[i]->getMissionIndex()+1,pos);
                    for(size_t i2 = Drones[i]->getMissionIndex()+2; i2< Drones[i]->getPath().size(); i2++){
                        Drones[i]->getPath()[i2].altitude=pos.altitude;
                    }
                }
            }
            if(changed){
                if(Drones[i]->getJob()->getStatus() == job::ongoing){
                    Drones[i]->getJob()->setStatus(job::resumeFlight);
                }else{
                    ROS_ERROR("[Ground Control]: Drone can't handle drone deconflic if not in ongoing state");
                }     
            }
        }
    }

}

// ################################### Main Program ###########################################
void initialize(void){
    nh = new ros::NodeHandle();

    RouteRequest_pub = nh->advertise<gcs::DronePath>("/gcs/forwardPath",100);
    ETA_pub = nh->advertise<gcs::DroneSingleValue>("/gcs/ETA",100);
    WebInfo_pub = nh->advertise<std_msgs::String>("/internet/ToInternet",100);
    JobState_pub = nh->advertise<gcs::DroneSingleValue>("/gcs/JobState",100);
    Heartbeat_pub = nh->advertise<node_monitor::heartbeat>("/node_monitor/input/Heartbeat",100);
    Reposition_pub = nh->advertise<gcs::moveTo>("/gcs/rePosition",100);
    Transport_pub = nh->advertise<gcs::DroneSingleValue>("/gcs/medicalTransport",100);
    RegisterDrone_pub = nh->advertise<drone_decon::RegisterDrone>("/drone_decon/register",100);

    Collision_sub = nh->subscribe("/collision_detector/collision_warning",100,Collision_Handler);
    WebInfo_sub = nh->subscribe("/internet/FromInternet",100,WebInfo_Handler);
    DroneStatus_sub = nh->subscribe("/drone_handler/DroneInfo",100,DroneStatus_Handler);
    nodeMonitor_sub = nh->subscribe("/node_monitor/node_list",10,nodeMonitor_Handler);
    drone_decon_sub = nh->subscribe("drone_decon/redirect",10,deconflict_Handler);
    
    
    

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
    GPSdistanceClient = nh->serviceClient<gcs::gps2distance>("/pathplan/GPS2GPSdist");
    safeTakeOffClient = nh->serviceClient<gcs::safeTakeOff>("/collision_detector/safeTakeOff");

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
        NodeState(node_monitor::heartbeat::nothing,"");

        // ############ Find Available drones for queued Jobs ############
        if(jobQ.size() != 0){
            for(size_t i = 0; i < Drones.size();i++){
                if(Drones[i]->isAvailable()){
                    if(GPSdistance(Drones[i]->getPosition(),jobQ.front()->getGoal()->getPosition())<15){
                        NodeState(node_monitor::heartbeat::warning,"Goal is right next to drone, no drone will be sent");
                        webMsg(Drones[i]->getJob()->getQuestHandler(),"request=failed,error=you already have a drone");
                    }else{
                        activeJobs.push_back(jobQ.front());
                        jobQ.pop_front();
                        Drones[i]->setJob(activeJobs.back());
                        Drones[i]->setAvailable(false);
                        Drones[i]->getJob()->setStatus(job::wait4pathplan);
                        webMsg(Drones[i]->getJob()->getQuestHandler(),"request=pathplaning");
                    }
                }
            }
        }

        // ############## Active Jobs look through ###################
        for(size_t i = 0; i < activeJobs.size();i++){
            uint8 status = activeJobs[i]->getStatus();
            if(status==job::wait4pathplan){ // ############# Do path planing for all jobs waiting for new Path plan ################
                gcs::GPS start = activeJobs[i]->getDrone()->getPosition();
                gcs::GPS end = activeJobs[i]->getGoal()->getPosition();
                // always start and stop above docking stations
                start.altitude = 32;
                end.altitude = 32;
                try{
                    std::vector<gcs::GPS> path = pathPlan(start, end,activeJobs[i]->getDrone());
                    activeJobs[i]->getDrone()->setPath(path);
                    activeJobs[i]->setStatus(job::preFlightCheack);
                    webMsg(activeJobs[i]->getQuestHandler(),"request=waiting4preflightCheck");
                }catch( string msg){
                    NodeState(node_monitor::heartbeat::critical_error,msg);
                }

            }else if(status == job::preFlightCheack){ // ### Pre FLight Cheacks ##################
                
                if(DO_PREFLIGHT_CHECK){
                    bool allOkay = true;

                    // ######### Cheack that UTM is running ################
                    if( !(utm_parser.ok == node_monitor::nodeOk::fine && 
                        (utm_parser.nodeState == node_monitor::heartbeat::info ||
                        utm_parser.nodeState == node_monitor::heartbeat::nothing)))
                    {
                        allOkay = false;
                        heartbeat_msg.severity = node_monitor::heartbeat::info;
                        heartbeat_msg.text = "Prefligt Check Failed, UTM not Okay";
                    }

                    // ######### Cheack that we are not inside no flight zone #########
                    is_safe_for_takeoff safe = safeTakeOff(activeJobs[i]->getDrone());
                    if(!safe.takeoff_is_safe){
                        allOkay = false;
                        NodeState(node_monitor::heartbeat::info,"Waiting for no FlightZone to clear before takeOff");
                    }

                    if(allOkay){
                        uploadFlightPlan(activeJobs[i]->getDrone());
                        activeJobs[i]->setStatus(job::ready4takeOff);
                        webMsg(activeJobs[i]->getQuestHandler(),"request=ready4takeoff");
                    }


                }else{
                    uploadFlightPlan(activeJobs[i]->getDrone());
                    activeJobs[i]->setStatus(job::ready4takeOff);
                    webMsg(activeJobs[i]->getQuestHandler(),"request=ready4takeoff");
                } 
                if(!activeJobs[i]->getGoal()->isLab()){
                    gcs::DroneSingleValue msg;
                    msg.value = 22;
                    msg.text = "Flying out to fetch blood";
                    msg.drone_id = activeJobs[i]->getDrone()->getID();
                    Transport_pub.publish(msg);
                }else{
                    gcs::DroneSingleValue msg;
                    msg.value = 3;
                    msg.text = "Flying back with blood";
                    msg.drone_id = activeJobs[i]->getDrone()->getID();
                    Transport_pub.publish(msg);
                }   
            
            
            }else if(status == job::done){  // ############# Delete Finnished Jobs ######################
                delete activeJobs[i];
                activeJobs.erase(activeJobs.begin()+i);
                i--;
            }else if(status == job::ongoing){ // ########### Send out INFO on Drone ETA ######################
                if( spins >= rate){
                    spins = 0;
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
            }else if(status == job::rePathPlan){ // ######## rePlan route ################
                try{
                    DNFZinject d = activeJobs[i]->getDNFZinjection();
                    if(d.stillValid){
                        if(d.valid_to-time(nullptr)>0){
                            std::vector<gcs::GPS> path = pathPlan(d.start, d.to,activeJobs[i]->getDrone());
                            std::vector<gcs::GPS> newPath;
                            std::vector<gcs::GPS> currentPath = activeJobs[i]->getDrone()->getPath();
                            std::vector<gcs::GPS> oldPath = activeJobs[i]->getOldPlan().plan;
                            int mission_index = activeJobs[i]->getDrone()->getMissionIndex();

                            size_t items = path.size();
                            items+= currentPath.size()-mission_index;
                            //items+= oldPath.size()-d.index_to;

                            newPath.reserve(items);
                            if( mission_index < d.index_from)
                                newPath.insert(newPath.end(),currentPath.begin()+mission_index,currentPath.begin()+d.index_from);
                            newPath.insert(newPath.end(),path.begin(),path.end());
                            //newPath.insert(newPath.end(),oldPath.begin()+d.index_to,oldPath.end());

                            activeJobs[i]->getDrone()->setPath(newPath);
                            uploadFlightPlan(activeJobs[i]->getDrone());
                            activeJobs[i]->setStatus(job::ready4flightContinuation); 

                        }else{
                            activeJobs[i]->setStatus(job::waitInAir);
                            activeJobs[i]->setWaitInAirTo(d.valid_to);
                            activeJobs[i]->setNextStatus(job::resumeFlight);
                        }
                        d.stillValid = false;
                        activeJobs[i]->DNFZinjection(d);
                    }else{
                        std::vector<gcs::GPS> path = pathPlan(activeJobs[i]->getDrone()->getPosition(), activeJobs[i]->getGoal()->getPosition(),activeJobs[i]->getDrone());
                        activeJobs[i]->getDrone()->setPath(path);
                        uploadFlightPlan(activeJobs[i]->getDrone());
                        activeJobs[i]->setStatus(job::ready4flightContinuation);         
                    }
                }catch( string msg){
                    NodeState(node_monitor::heartbeat::critical_error,msg);
                }
            }else if(status == job::waitInAir){ // ######### waiting for somthing ############
                if(activeJobs[i]->getWaitTime() != -1 && std::time(nullptr) > activeJobs[i]->getWaitTime()){
                    activeJobs[i]->setWaitInAirTo(-1);
                    activeJobs[i]->setStatus(activeJobs[i]->getNextStatus());
                    activeJobs[i]->setNextStatus(job::notAssigned);
                }else{
                    ROS_ERROR("[Ground Control]: No continuation to this waiting state implemented");
                }
                //Check that we can move to next waypoint
            }else if(status == job::notAssigned){
                ROS_ERROR("[Ground Control]: somthing unexpected happened and the job is without a valid status");
            }else if(status == job::resumeFlight){
                //TODO check that we can move to next waypoint
                drone* theDrone = activeJobs[i]->getDrone();
                vector<gcs::GPS> path(theDrone->getPath().begin()+theDrone->getMissionIndex(),theDrone->getPath().end());
                uploadFlightPlan(theDrone);
                activeJobs[i]->setStatus(job::ready4flightContinuation);
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
                }else if(msg.value == job::preFlightCheack){
                    msg.text = "Waiting for all clear signal";
                }else if(msg.value == job::rePathPlan){
                    msg.text = "rePlanPath";
                }if(msg.value == job::waitInAir){
                    msg.text = "Wait in air";
                }if(msg.value == job::resumeFlight){
                    msg.text = "resumeFlight from old plan";
                }if(msg.value == job::ready4flightContinuation){
                    msg.text = "ready to continue";
                }
            }else{
                msg.value = job::noMission;
                msg.text = "No Job assigned";
            }
            JobState_pub.publish(msg);
        }

        spins++;
    }

    delete nh;
}

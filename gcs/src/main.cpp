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

#include <signal.h>
#include <thread>
#include <atomic>

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
#define MIN_FLIGHT_DISTANCE 15
#define MAX_FLIGHT_DISTANCE 500

#define DEFAULT_FLIGHT_HEIGHT 32
#define FORWAD_CHECK_DISTANCE 15
#define earthRadiusKm 6371.0

#define UTM_EMERGENCY 0
#define UTM_MEDICAL 3
#define UTM_TRANSPORTER 21
#define UTM_GROUNDED 22

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

int pathPlanNum =0;
const string header = "[Ground Control]: ";

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
double deg2rad(double deg) {
  return (deg * M_PI / 180);
}
double rad2deg(double rad) {
  return (rad * 180 / M_PI);
}

string jobStatusText(int status){
    string text = "";
    if(status == job::queued){
        text = "Queued";
    }else if(status == job::ongoing){
        text = "Ongoing";
    }else if(status == job::onhold){
        text = "Onhold";
    }else if(status == job::wait4pathplan){
        text = "Waiting for pathplan";
    }else if(status == job::ready4takeOff){
        text = "Ready for takeoff";
    }else if(status == job::done){
        text = "Done";
    }else if(status == job::preFlightCheack){
        text = "Waiting for all clear signal";
    }else if(status == job::rePathPlan){
        text = "rePlanPath";
    }if(status == job::waitInAir){
        text = "Wait in air";
    }if(status == job::resumeFlight){
        text = "resumeFlight from old plan";
    }if(status == job::ready4flightContinuation){
        text = "ready to continue";
    }
    return text;
}
// ############################ Helper functions ##################################
void UTMPriority(int priority, drone* aDrone, string description = ""){
    gcs::DroneSingleValue msg;
    msg.value = priority;
    msg.text = description;
    msg.drone_id = aDrone->getID();
    if(priority == UTM_EMERGENCY){
        aDrone->OS()=drone::emergency;
    }
    Transport_pub.publish(msg);
}

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
                 text.getValue("lab") == "true",
                 text.getValue("EM") == "true"
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
double GPSdistance(const gcs::GPS &point1, const gcs::GPS &point2 ,bool print_dist = true);
dock* closestLab(drone* theDrone){ // TODO
    double distance = INFINITY;
    dock* theLab = NULL;
    if(theDrone != NULL){
        for(size_t i = 0; i < Docks.size(); i++){
            if(Docks[i]->isLab()){
                double d = GPSdistance(Docks[i]->getPosition(),theDrone->getPosition());
                if(d < distance){
                    distance = d;
                    theLab = Docks[i];
                }
            }
        }
    }
    return theLab;
}
bool safePosition(gcs::GPS, drone*);
dock* findEmergencyLand(drone* theDrone){
    double distance = INFINITY;
    dock* EMLandingPAD;
    for(size_t i = 0; i < Docks.size(); i++){
        if(Docks[i]->isEM() && (safePosition(Docks[i]->getPosition(),theDrone) || theDrone->OS() == drone::emergency)){
            double d = GPSdistance(Docks[i]->getPosition(),theDrone->getPosition());
            if(d < distance){
                distance = d;
                EMLandingPAD = Docks[i];
            }
        }
    }
    return EMLandingPAD;
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
void uploadFlightPlan(drone* theDrone,bool loiterAtEnd = false){
    gcs::DronePath msg;

    msg.DroneID = theDrone->getID();
    msg.loiterAtEnd = loiterAtEnd;
    msg.PathID = pathPlanNum++;
    if(DEBUG){
        while(theDrone->getPath().size()> 9){
            theDrone->getPath().pop_back();
            msg.loiterAtEnd = true;
        }   
    }
    if(std::abs(theDrone->getFlightHeight()-theDrone->getPath().back().altitude)>0.1){
        for(size_t i = 0; i < theDrone->getPath().size();i++){
            theDrone->getPath()[i].altitude=theDrone->getFlightHeight();
        }
    }
    if(DEBUG) cout << "[Ground Control]: Posting PathPlan - length:" << theDrone->getPath().size() << endl;
    msg.Path = theDrone->getPath();
    RouteRequest_pub.publish(msg);
}
void moveDroneTo(drone *theDrone, gcs::GPS pos, long waitTill = -1){
    job* theJob =  theDrone->getJob();
    if(theJob != NULL){
        cout << "[Ground Control]: Asking for reposition" << endl;
        theDrone->getPath()={pos};
        uploadFlightPlan(theDrone,true);
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
dock* findDock(string name){
    dock* goalDock = NULL;
    for(size_t i = 0; i < Docks.size(); i++){
        if(Docks[i]->getName() == name){
            if(DEBUG) cout << "[Ground Control]: " << "Dock found" << endl;
            goalDock = Docks[i];
        }
    }
    return goalDock;
}
void landEmergency(drone* theDrone){
    NodeState(node_monitor::heartbeat::critical_error,"Doing emegency landing");
    UTMPriority(UTM_EMERGENCY, theDrone);
    theDrone->getPath()= {theDrone->getPosition()};
    if(theDrone->getJob() != NULL){
        theDrone->getJob()->terminateJobOnLand() = true;
    }
    uploadFlightPlan(theDrone);
}
// ############################ Service calls ##################################
double GPSdistance(const gcs::GPS &point1, const gcs::GPS &point2, bool print_dist){
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = deg2rad(point1.latitude);
    lon1r = deg2rad(point1.longitude);
    lat2r = deg2rad(point2.latitude);
    lon2r = deg2rad(point2.longitude);
    u = sin((lat2r - lat1r)/2);
    v = sin((lon2r - lon1r)/2);
    double dist = 1000*(2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v)));
    if(print_dist) cout << "[Ground Control]: " << "Distance calculated " << dist << endl;
    return dist;
}
is_safe_for_takeoff safeTakeOff(drone* my_drone){
    gcs::safeTakeOff srv;
    srv.request.drone_id = my_drone->getID();
    srv.request.takeoff_position = my_drone->getPosition();
    bool worked = safeTakeOffClient.call(srv);

    is_safe_for_takeoff response;
    if (worked){
        if(DEBUG) cout << "[Ground Control]: " << "SafeTake off is " << int(srv.response.is_save_to_take_off) << " time to clear " << int(srv.response.time_to_clear) << " current epoch time " <<  std::time(nullptr) << endl;
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
bool safePosition(gcs::GPS position,drone* my_drone){
    gcs::safeTakeOff srv;
    srv.request.drone_id = my_drone->getID();
    srv.request.takeoff_position = my_drone->getPosition();
    bool worked = safeTakeOffClient.call(srv);
    bool response;
    if (worked){
        //if(DEBUG) std::cout << "[Ground Control]: " << "is point inside DNFZ? : " << !int(srv.response.is_save_to_take_off) << endl;
        response = srv.response.is_save_to_take_off;
    }else{
        cout << "[Ground Control]: " << "Safe point cheack failed"<< endl;
        NodeState(node_monitor::heartbeat::critical_error,"Safe point cheack failed");
        response= false;
    }
    return response;
}
long ETA(job* aJob){
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
        return 0;
    }
    return srv.response.eta;
}
std::vector<gcs::GPS> pathPlan(gcs::GPS start,gcs::GPS end,drone* theDrone){

    gcs::pathPlan srv;
    srv.request.start = start;
    srv.request.end = end;
    srv.request.drone_id = theDrone->getID();

    if(theDrone->getJob() != NULL){
        if(theDrone->getJob()->getStatus() == job::wait4pathplan || theDrone->OS() != drone::normal_operation){
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

    if((srv.response.path.size() < 2 && theDrone->getJob()->getStatus() == job::wait4pathplan) || !worked || srv.response.path.size() < 1 ){
        ROS_ERROR("[Ground Control]: Unacceptable Path");
        throw string("Unacceptable Path");
    }
    if(theDrone->OS() == drone::emergency_plan_with_fallback){
        theDrone->OS() == drone::normal_operation;
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
        Drones[index]->setMissionIndex(msg.mission_index);
        Drones[index]->setVelocity(msg.ground_speed);
        Drones[index]->setVelocitySetPoint(msg.ground_speed_setpoint);
        Drones[index]->setStatus(msg.status);
        Drones[index]->setHeading(msg.heading);
        if(msg.status == msg.Run){
            if(Drones[index]->getJob() != NULL){
                if(Drones[index]->getJob()->getStatus()==job::ongoing){
                    if(!safePosition(Drones[index]->forwardPosition(FORWAD_CHECK_DISTANCE),Drones[index])){
                        cout << "Forward position is inside NFZ" << endl;
                        if(GPSdistance(Drones[index]->getPosition(),Drones[index]->getPath()[msg.mission_index])>FORWAD_CHECK_DISTANCE){
                            moveDroneTo(Drones[index],Drones[index]->getPosition());
                            Drones[index]->getJob()->setStatus(job::rePathPlan);
                        }
                    }
                }
            }
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
                if(aJob->getTerminateJobOnLand() && aJob->getStatus() == job::onhold){
                    cout << "[Ground Control]: Terminate Job on land is set: " << int(aJob->getTerminateJobOnLand()) << " - setting job to done from: " << int(aJob->getStatus()) << endl;
                    aJob->setStatus(job::done);
                }else if(aJob->getStatus() == job::ongoing){
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
                }else{
                    cout << "[Ground Control]: Drone status = hold, job status is: " << jobStatusText(int(aJob->getStatus())) << endl; 
                }
            }else{
                ROS_ERROR("[Ground Control]: Drone in Hold without a JOB");
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
            dock* goalDock = findDock(msg.getValue("name"));       
            if(goalDock != NULL){
                if(DEBUG) cout << "[Ground Control]: " << "making job" << endl;
                jobQ.push_back(new job(goalDock));
                if(msg.getValue("request")=="terminate"){
                    cout << "[Ground Control]: " << "job is set to terminate on land" << endl;
                    jobQ.back()->terminateJobOnLand() = true;
                }
                feedback+= ",request=queued";
            }else{
                feedback+= ",request=failed,error=No Dockingstation named " + msg.getValue("name");
            }
            if(DEBUG) cout << "[Ground Control]: " << "request end" << endl;
        }else if(msg.hasValue("return")){  // ##########  RETURN ###############
            bool foundJob = false;
            for(size_t i = 0; i < activeJobs.size();i++){
                if(activeJobs[i]->getStatus() == job::onhold){
                    if(activeJobs[i]->getQuestHandler()->getName() == msg.getValue("name")){
                        dock* goalDock = NULL;
                        foundJob = true;
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
                            }else if(GPSdistance(Drones[i]->getPosition(),goalDock->getPosition())>500){
                                NodeState(node_monitor::heartbeat::warning,"Goal is too far away from drone");
                                webMsg(Drones[i]->getJob()->getQuestHandler(),"request=failed,error=goal too far away");
                                break;
                            }
                        }
                        
                        activeJobs[i]->setGoal(goalDock);
                        activeJobs[i]->setStatus(job::wait4pathplan);
                        feedback+=",return=succes,request=waiting4pathplan";
                        break;
                    }
                }else{
                    webMsg(activeJobs[i]->getQuestHandler(),"return=drone not arrived yet");
                }
            }
            if(!foundJob){
                dock* aDock = findDock(msg.getValue("name"));
                if(aDock!=NULL){
                    for(size_t i = 0; i < Drones.size(); i++){
                        if(Drones[i]->isAvailable() && Drones[i]->getStatus() == gcs::DroneInfo::Land){
                            if(GPSdistance(Drones[i]->getPosition(),aDock->getPosition())<15){
                                Drones[i]->setAvailable(false);
                                job* aJob = new job(aDock);
                                aJob->setStatus(job::onhold);
                                aJob->setDrone(Drones[i]);
                                activeJobs.push_back(aJob);
                                WebInfo_Handler(msg_in);
                            }
                        }
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
        if(aDrone->getID() == msg.drone_id && activeJobs[i]->getStatus()!= job::onhold){
            if(!activeJobs[i]->getDNFZinjection().stillValid&&
                (activeJobs[i]->getDNFZinjection().dnfz_id != msg.dnfz_id ||
                long(std::time(nullptr))-activeJobs[i]->getDNFZinjection().time > 30))
            {
                cout << "[Ground Control]: DNFZ is valid : " << activeJobs[i]->getDNFZinjection().stillValid << endl;
                cout << "[Ground Control]: DNFZ ID       : " << activeJobs[i]->getDNFZinjection().dnfz_id << " and " << msg.dnfz_id << endl;
                cout << "[Ground Control]: DNFZ time diff: " <<  long(std::time(nullptr))-activeJobs[i]->getDNFZinjection().time << endl;
                if(GPSdistance(aDrone->getPosition(),activeJobs[i]->getGoal()->getPosition())>2){
                    if(msg.zone_type == gcs::inCollision::normal_zone){
                        cout << "[Ground Control]: Handeling Normal DNFZ zone" << endl;
                        activeJobs[i]->DNFZinjection(msg);
                        activeJobs[i]->getDNFZinjection().to = activeJobs[i]->getGoal()->getPosition(); 
                        activeJobs[i]->saveOldPlan();
                        vector<gcs::GPS> path = aDrone->getPath();
                        vector<gcs::GPS> newPath(path.begin()+aDrone->getMissionIndex(),path.begin()+msg.plan_index1);
                        newPath.push_back(msg.start);
                        aDrone->setPath(newPath);
                        uploadFlightPlan(aDrone,true);
                        activeJobs[i]->setStatus(job::rePathPlan);

                    }else if(msg.zone_type == gcs::inCollision::inside_zone){
                        cout << "[Ground Control]: Handeling inside DNFZ zone" << endl;
                        activeJobs[i]->DNFZinjection(msg);
                        activeJobs[i]->setStatus(job::rePathPlan);
                        activeJobs[i]->saveOldPlan();

                        moveDroneTo(activeJobs[i]->getDrone(),msg.start);

                    }else if(msg.zone_type == gcs::inCollision::landing_zone){
                        cout << "[Ground Control]: Handeling landing DNFZ zone" << endl;
                        moveDroneTo(activeJobs[i]->getDrone(),activeJobs[i]->getDrone()->getPosition());
                        activeJobs[i]->DNFZinjection(msg);
                        activeJobs[i]->getDNFZinjection().start = activeJobs[i]->getDrone()->getPosition();
                        activeJobs[i]->getDNFZinjection().to = findEmergencyLand(activeJobs[i]->getDrone())->getPosition();
                        cout << "[Ground Control]: Emergency landing terminate Job on land" << endl;
                        activeJobs[i]->terminateJobOnLand() = true;
                        activeJobs[i]->setStatus(job::rePathPlan);

                        //ROS_ERROR("[Ground Control]: can't handle obstructed ladingzone yet");
                    }else{
                        ROS_ERROR("[Ground Control]: Error unrecognized dnfz type");
                    }
                }else{
                    cout << "[Ground Control]: Too colse to target DNFZ ignored" << endl;
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
            if(Drones[i]->getStatus() != gcs::DroneInfo::Land){
                gcs::GPS pos;
                pos << msg.position;
                pos.altitude-= Drones[i]->getGroundHeight();
                if(pos.altitude < 5) pos.altitude = 5;
                bool changed = false;
                if(Drones[i]->getPath().size() > Drones[i]->getMissionIndex()+1){
                    // ###################### NORMAL DRONE_DECON ######################################
                    if(msg.insertBeforeNextWayPoint){
                        if(GPSdistance(pos,Drones[i]->getPath()[Drones[i]->getMissionIndex()])>5 &&
                            std::abs(pos.altitude-Drones[i]->getPath()[Drones[i]->getMissionIndex()].altitude)>1)
                        {
                            changed = true;
                            Drones[i]->getPath().insert(Drones[i]->getPath().begin()+Drones[i]->getMissionIndex(),pos);
                            for(size_t i2 = Drones[i]->getMissionIndex()+1; i2< Drones[i]->getPath().size(); i2++){
                                Drones[i]->getPath()[i2].altitude=pos.altitude;
                            }
                        }
                    }else{
                        if(GPSdistance(pos,Drones[i]->getPath()[Drones[i]->getMissionIndex()+1])>5 &&
                            std::abs(pos.altitude-Drones[i]->getPath()[Drones[i]->getMissionIndex()+1].altitude)>1)
                        {
                            changed = true;
                            Drones[i]->getPath().insert(Drones[i]->getPath().begin()+Drones[i]->getMissionIndex()+1,pos);
                            for(size_t i2 = Drones[i]->getMissionIndex()+2; i2< Drones[i]->getPath().size(); i2++){
                                Drones[i]->getPath()[i2].altitude=pos.altitude;
                            }
                        }
                    }



                    // ########################## IREGULAR DROEN_DECON ####################################
                }else{
                    ROS_WARN("[Ground Control]: Somthing wrong with pathPlan or mission INDEX in deconflict handler");
                    if(std::abs(pos.altitude-Drones[i]->getPosition().altitude)>1){
                        Drones[i]->getFlightHeight() = pos.altitude;
                        uint8 jobStatus = Drones[i]->getJob()->getStatus();
                        if(jobStatus == job::ongoing || jobStatus == job::waitInAir){
                            ROS_WARN("[Ground Control]: Attemting to move to drone_decon position");
                            if(safePosition(pos,Drones[i])){
                                ROS_INFO("[Ground Control]: Moving to position");
                                moveDroneTo(Drones[i],pos);
                            }else{
                                gcs::GPS newPos = Drones[i]->getPosition();
                                newPos.altitude = pos.altitude;
                                moveDroneTo(Drones[i],newPos);
                                ROS_ERROR("[Ground Control]: Move was not safe, adjusting current positions hegit");
                            }      
                        }else{
                            ROS_ERROR("[Ground Control]: Drone is trying to make a new pathplan so a flag to adjust the height has been added");
                        }
                    }else{
                        ROS_INFO("[Ground Control]: doesen't matter drone is at correct height");
                    }                   
                }

                // ##################################### Aplying the changes ####################################
                if(changed){
                    if(Drones[i]->getJob() != NULL){
                        if(Drones[i]->getJob()->getStatus() == job::ongoing){
                            Drones[i]->getFlightHeight() = pos.altitude;
                            Drones[i]->getJob()->setStatus(job::resumeFlight);
                            cout << "[Ground Control]: drone_deconflict moved drone" << endl;
                        }else{
                            Drones[i]->getFlightHeight() = pos.altitude;
                            ROS_WARN("[Ground Control]: Drone is in the middle of somthing setting flag to change height at next path upload");
                            //ROS_ERROR("[Ground Control]: Drone can't handle drone deconflic if not in ongoing state");
                        }
                    }else{
                        ROS_ERROR("[Ground Control]: Can't handle drone without JOB");
                    }
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
    NodeState(node_monitor::heartbeat::nothing,"",0.3);
    sleep(3);
    NodeState(node_monitor::heartbeat::info,"Waiting for password",0.05);
    internet::getIp srv;
    srv.request.username = "waarbubble@gmail.com";
    cout << "[Ground Control]: webServer Username: " << srv.request.username <<endl;
    //cout << "[Ground Control]: webServer Password: " << flush;
    

    //Below cin operation should be executed within stipulated period of time

    
    string password;
    cout << "[Ground Control]: webServer Password: " << flush;
    getline(cin, password);
    if(password.size()>2){
        srv.request.password = password;
        bool worked = false;
        const long maxTries = 3;
        long tries = 0;
        cout << "[Ground Control]: " << "Getting Server IP" << endl;
        NodeState(node_monitor::heartbeat::info,"Getting Server IP",0.05);
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
                NodeState(node_monitor::heartbeat::info,"Waiting for password",0.05);
                getline(cin, password);
            }
        }
        if(tries == maxTries){
            ROS_ERROR("Could not obtain IP");
        }
    }else{
        cout << "[Ground Control]: ignoring IP service" << endl;
    }


    


    
}


int main(int argc, char** argv){
    ros::init(argc,argv,"gcs");
    heartbeat_msg.header.frame_id ="gcs";

    initialize();
    unsigned long spins = 0;

    int rate = 10;
    ros::Rate r(rate);
    heartbeat_msg.rate = rate;
    heartbeat_msg.severity = node_monitor::heartbeat::nothing;

    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
        NodeState(node_monitor::heartbeat::nothing,"");

        // ############ Find Available drones for queued Jobs ############
        if(jobQ.size() != 0){
            for(size_t i = 0; i < Drones.size();i++){
                if(Drones[i]->isAvailable()){
                    cout << header << "Available drone found for job by: " << jobQ.front()->getQuestHandler()->getName() << endl;
                    double goal2drone =GPSdistance(Drones[i]->getPosition(),jobQ.front()->getGoal()->getPosition());
                    if(goal2drone<MIN_FLIGHT_DISTANCE){
                        NodeState(node_monitor::heartbeat::warning,"Goal is right next to drone, no drone will be sent");
                        webMsg(jobQ.front()->getQuestHandler(),"request=failed,error=you already have a drone");
                        jobQ.pop_front();
                    }else if(goal2drone>MAX_FLIGHT_DISTANCE){ 
                        NodeState(node_monitor::heartbeat::warning,"Goal is too far away from drone");
                        webMsg(jobQ.front()->getQuestHandler(),"request=failed,error=goal too far away");
                        //TODO for full system implementation push back job in queue
                        jobQ.pop_front();
                    }else{
                        cout << header << "Job assigned to drone: " << Drones[i]->getID() << endl;
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
            if(status == job::notAssigned){
                ROS_ERROR("[Ground Control]: somthing unexpected happened and the job is without a valid status");
            }else if(status == job::noMission){

            }else if(status == job::queued){

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
            }else if(status == job::onhold){

            }else if(status==job::wait4pathplan){ // ############# Do path planing for all jobs waiting for new Path plan ################
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

            }else if(status==job::ready4takeOff){

            }else if(status == job::done){  // ############# Delete Finnished Jobs ######################           
                webMsg(activeJobs[i]->getQuestHandler(),"return=done");
                delete activeJobs[i];
                activeJobs.erase(activeJobs.begin()+i);
                i--;
            }else if(status == job::preFlightCheack){ // ### Pre FLight Cheacks ##################

                if(DO_PREFLIGHT_CHECK){
                    bool allOkay = true;

                    // ######### Cheack that UTM is running ################
                    /*if( !(utm_parser.ok == node_monitor::nodeOk::fine &&
                        (utm_parser.nodeState == node_monitor::heartbeat::info ||
                        utm_parser.nodeState == node_monitor::heartbeat::nothing)))
                    {
                        allOkay = false;
                        heartbeat_msg.severity = node_monitor::heartbeat::info;
                        heartbeat_msg.text = "Prefligt Check Failed, UTM not Okay";
                    }*/

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


            }else if(status == job::rePathPlan){ // ######## rePlan route ################
                try{
                    DNFZinject d = activeJobs[i]->getDNFZinjection();
                    if(d.stillValid){
                        if(d.valid_to-time(nullptr)>0){
                            /*cout << header << "DNFZ re pathplan" << endl;
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
                            */
                        }else{
                            cout << header << "Wait and Resume path" << endl;
                            activeJobs[i]->setStatus(job::waitInAir);
                            activeJobs[i]->setWaitInAirTo(d.valid_to);
                            activeJobs[i]->setNextStatus(job::resumeFlight);
                        }
                        d.stillValid = false;
                        activeJobs[i]->DNFZinjection(d);
                    }else{
                        cout << header << "Normal rePathPlan" << endl;
                        std::vector<gcs::GPS> path = pathPlan(activeJobs[i]->getDrone()->getPosition(), activeJobs[i]->getGoal()->getPosition(),activeJobs[i]->getDrone());
                        if(path.size()> 2){
                            path.erase(path.begin());
                        }
                        activeJobs[i]->getDrone()->setPath(path);
                        uploadFlightPlan(activeJobs[i]->getDrone());
                        activeJobs[i]->setStatus(job::ready4flightContinuation);
                    }
                }catch( string msg){
                    NodeState(node_monitor::heartbeat::critical_error,msg);
                    cout << header << "Somthing went wrong in path planing trying aomthing else" << endl;
                    if(activeJobs[i]->getDNFZinjection().stillValid == false){
                        if(safePosition(activeJobs[i]->getDrone()->getPosition(),activeJobs[i]->getDrone())){
                            cout << header << "The drone is planning to goal and having trouble, finding emergancy landing spot" << endl;
                            dock* EM = findEmergencyLand(activeJobs[i]->getDrone());
                            if(EM != NULL){
                                cout << header << "Trying to land at: "<< EM->getName();
                                activeJobs[i]->setGoal(EM);
                                activeJobs[i]->terminateJobOnLand() = true;
                            }else{
                                UTMPriority(UTM_EMERGENCY,activeJobs[i]->getDrone());
                                dock* EM = findEmergencyLand(activeJobs[i]->getDrone());
                                if(EM == NULL){
                                    ROS_ERROR("Couldn't find solution to missing path plan landing right here");
                                    landEmergency(activeJobs[i]->getDrone());
                                }else{
                                    cout << header << "Setting emergancy state and moving towards Emergency landing spot: " << EM->getName() << endl;
                                    moveDroneTo(activeJobs[i]->getDrone(),EM->getPosition());
                                }
                            }
                        }else{
                            cout << header << "We are currently inside a DNFZ and the planner can't find a path" << endl;
                            if(activeJobs[i]->getDrone()->OS() == drone::normal_operation){
                                cout << header << "Elevating drone to ignore DNFZ for next pathPlan" << endl;
                                activeJobs[i]->getDrone()->OS() = drone::emergency_plan_with_fallback;
                            }else{
                                dock* EM = findEmergencyLand(activeJobs[i]->getDrone());
                                if(EM == NULL){
                                    cout << header << "Couldn't find Emergency landing spot, moving to closest lab" << endl;
                                    dock* lab = closestLab(activeJobs[i]->getDrone());
                                    if(lab != NULL){
                                        UTMPriority(UTM_EMERGENCY,activeJobs[i]->getDrone());
                                        moveDroneTo(activeJobs[i]->getDrone(),lab->getPosition());
                                    }else{
                                        UTMPriority(UTM_EMERGENCY,activeJobs[i]->getDrone());
                                    }
                                }else{
                                    cout << header << "Setting emergancy state and moving towards Emergency landing spot: " << EM->getName() << endl;
                                    UTMPriority(UTM_EMERGENCY,activeJobs[i]->getDrone());
                                    moveDroneTo(activeJobs[i]->getDrone(),EM->getPosition());

                                }
                            }
                            
                        }
                    }else{
                        cout <<  header << "Tried a DNFZ repathPlan didn't work, trying to plan to home" << endl;
                        activeJobs[i]->getDNFZinjection().stillValid = false;
                    }
                    
                }
            }else if(status == job::waitInAir){ // ######### waiting for somthing ############
                if(activeJobs[i]->getWaitTime() != -1 && std::time(nullptr) > activeJobs[i]->getWaitTime()){
                    activeJobs[i]->setWaitInAirTo(-1);
                    activeJobs[i]->setStatus(activeJobs[i]->getNextStatus());
                    activeJobs[i]->setNextStatus(job::notAssigned);
                }else if(activeJobs[i]->getNextStatus() != job::notAssigned){
                    activeJobs[i]->setStatus(activeJobs[i]->getNextStatus());
                    activeJobs[i]->setNextStatus(job::notAssigned);
                }else{
                    ROS_ERROR("[Ground Control]: No continuation to this waiting state implemented, flying to goal");
                    activeJobs[i]->getDNFZinjection().stillValid = false;
                    activeJobs[i]->setStatus(job::rePathPlan);
                }
                //Check that we can move to next waypoint
            }else if(status == job::resumeFlight){
                //TODO check that we can move to next waypoint
                drone* theDrone = activeJobs[i]->getDrone();
                vector<gcs::GPS> path(theDrone->getPath().begin()+theDrone->getMissionIndex(),theDrone->getPath().end());
                uploadFlightPlan(theDrone);
                activeJobs[i]->setStatus(job::ready4flightContinuation);
            }else if(status == job::ready4flightContinuation){

            }else{
                ROS_ERROR(header.c_str(),"Invalid job status");
                drone* D = activeJobs[i]->getDrone();
                if(D->getStatus() == gcs::DroneInfo::Run){
                    activeJobs[i]->setStatus(job::resumeFlight);
                }else if(D->getStatus() == gcs::DroneInfo::Land){
                    activeJobs[i]->setStatus(job::done);
                }else{
                    activeJobs[i]->setStatus(job::waitInAir);
                }
            }
        }



        // #################### Job state pub ########################
        for(size_t i = 0 ; i < Drones.size(); i++){
            gcs::DroneSingleValue msg;
            msg.drone_id = Drones[i]->getID();
            if(Drones[i]->getJob() != NULL){
                msg.value = Drones[i]->getJob()->getStatus();
                msg.text = jobStatusText(msg.value);
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

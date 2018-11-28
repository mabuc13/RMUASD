
#include "DronesAndDocks.hpp"
#include <iostream>
#include <math.h>


gcs::GPS UTM2GPS(UTM coord){
  gcs::GPS ret;
  Geo::UTMtoLL(coord.north,coord.east,coord.zone,ret.latitude,ret.longitude);
  ret.altitude=coord.altitude;
  return ret;

}
UTM GPS2UTM(gcs::GPS coord){
  UTM ret;
  Geo::LLtoUTM(coord.latitude,coord.longitude,ret.north,ret.east,ret.zone);
  ret.altitude = coord.altitude;
  return ret;
}

//################## Dock ###################
dock::dock(string name,double latitude, double longitude, double altitude, bool isLab):
  name(name),isALab(isLab){
  position.longitude = longitude;
  position.latitude = latitude;
  position.altitude = altitude;
}
gcs::GPS dock::getPosition(void){
  return position;
}
string dock::getName(void){
  return name;
}
bool dock::isLab(void){
  return isALab;
}


//################# Job #########################
job::job(dock* station):worker(NULL),goal(NULL),QuestGiver(NULL),status(job::queued){
    goal = station;
    QuestGiver = station;
}
job::job(const job &ajob):worker(NULL){
    this ->goal = ajob.goal;
    this-> QuestGiver = ajob.QuestGiver;
    this->status = ajob.status;
    this->setDrone(ajob.worker);
}
job::~job(){
    this->worker->setJob(NULL);
}


uint8 job::getStatus(void){
    return status;
}
drone* job::getDrone(void){
    return worker;
}
dock* job::getQuestHandler(void){
    return QuestGiver;
}
dock* job::getGoal(void){
    return this->goal;
}

void job::setDrone(drone* theDrone){

    if(worker != NULL){
        worker ->setJob(NULL);
    }
    worker = theDrone;
    if(theDrone != NULL){
        if(theDrone->getJob() != this){
            theDrone->setJob(this);
        }
    }
}
void job::setGoal(dock* goal){
    this->goal=goal;
}
void job::setStatus(uint8 status){
    this->status = status;
    std::cout <<"[Ground control]: "<< "Job: " << this->getQuestHandler()->getName() << " - Status: " << (int)status << std::endl;
}

//################### SimpleDrone ######################
simpleDrone::simpleDrone():vel_list(4){}
simpleDrone::simpleDrone(ID_t ID, gcs::GPS curPos):
    vel_list(10),cur_pos(curPos),drone_id(ID){}

simpleDrone::simpleDrone(gcs::UTMDrone info):vel_list(4){
    
    this->update_values(info);
}
void simpleDrone::update_values(gcs::UTMDrone info){
    this->next_wp = info.next_WP;
    this->cur_pos = info.cur_pos;
    this->next_vel = info.next_vel;
    this->cur_vel = info.cur_vel;

    if(this->drone_id == 0){
        for(size_t i = 0; i < vel_list.size();i++){
            vel_list[i]= info.cur_vel;
            vel_acc+=info.cur_vel;
        }
    }
    this->vel_list.push_back(info.cur_vel);
    this->vel_acc = info.cur_vel - this->vel_list.front();
    this->vel_list.pop_front();
    this->cur_vel_est = this->vel_acc/this->vel_list.size();

    while(info.cur_heading > 360) info.cur_heading -= 360;
    while(info.cur_heading<0 ) info.cur_heading +=360;
    while(info.next_heading > 360) info.next_heading -= 360;
    while(info.next_heading<0 ) info.next_heading +=360;

    this->next_heading = info.next_heading;
    this->cur_heading = info.cur_heading;
    this->time = info.time;
    this->gps_time = info.gps_time;
    this->battery_soc = info.battery_soc;
    this->drone_priority = info.drone_priority;

    this->ETA_next_WP= info.ETA_next_WP;
    this->drone_id = info.drone_id;
}
gcs::GPS simpleDrone::getPosition(){return this->cur_pos;}
UTM simpleDrone::getPositionU(){return GPS2UTM(this->cur_pos);}
UTM simpleDrone::getNextPositionU(){return GPS2UTM(this->next_wp);}
direction simpleDrone::getHeading(double heading){
    double angle;  
    if(360>= heading && heading >= 270){
        angle = 450-heading;
    }else if (270>heading && heading > 90){
        angle -(heading-90);
    }else if(heading){
        angle = 90-heading;
    }
    angle = M_PI * angle/180;
    direction ret;
    ret.north = std::sin(angle);
    ret.east = std::cos(angle);

    return ret;
}
direction simpleDrone::getCurHeading(){return getHeading(this->cur_heading);}
direction simpleDrone::getNextHeading(){return getHeading(this->next_heading);}
double simpleDrone::getCurVelocity(){return this->cur_vel;}
double simpleDrone::getNextVelocity(){return this->next_vel;}
ID_t simpleDrone::getID(){return this->drone_id;}
double simpleDrone::getTime(){
    if(this->gps_time < 10)
        return this->time;
    return this->gps_time;
}
double simpleDrone::getEtaNextWP(){return this->ETA_next_WP;}
double simpleDrone::getBatterySOC(){return this->battery_soc;}
uint8_t simpleDrone::getPriority(){return this->drone_priority;}

//################### Drone ####################
drone::drone(ID_t ID, gcs::GPS position):
    ID(ID),
    position(position),
    isFree(true),
    currentJob(NULL)
{}
ID_t drone::getID(void){
    return this->ID;
}
gcs::GPS drone::getPosition(void){
    return this->position;
}
std::vector<gcs::GPS> drone::getPath(void){
    return this->thePath;
}
bool drone::isAvailable(void){
    return this->isFree;
}
job* drone::getJob(void){
    return currentJob;
}
double drone::getVelocity(){
    return velocity;
}
size_t drone::getMissionIndex(){
    return pathIndex;
}

void drone::setAvailable(bool avail){
    this->isFree = avail;
}
void drone::setPath(const std::vector<gcs::GPS> &path){
    this->thePath = path;
}
void drone::setJob(job* aJob){
    if(aJob != NULL){
        if (this->currentJob != NULL){
            this->currentJob->setDrone(NULL);
        }
        this->currentJob = aJob;
        if(aJob->getDrone() != this){
            aJob->setDrone(this);
        }
    }else{
        isFree = true;
        this->currentJob = NULL;
    }
}
void drone::setPosition(gcs::GPS position){
    this->position = position;
}
void drone::setVelocity(double v){
    this->velocity = v;
}
void drone::setMissionIndex(size_t i){
    this->pathIndex = i;
}

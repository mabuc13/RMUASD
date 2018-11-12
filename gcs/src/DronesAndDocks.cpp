
#include "DronesAndDocks.hpp"
#include <iostream>


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
job::job(dock* station):status(1),worker(NULL),goal(NULL),QuestGiver(NULL),status(job::noMission){
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

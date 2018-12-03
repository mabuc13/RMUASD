
#include "DronesAndDocks.hpp"
#include <iostream>
#include <math.h>
#include <cmath> 

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
    //std::cout <<"[Ground control]: "<< "Job: " << this->getQuestHandler()->getName() << " - Status: " << (int)status << std::endl;
}

void job::setNextStatus(uint8 status){
    this->nextStatus = status;
}
uint8 job::getNextStatus(void){
    return this->nextStatus;
}

void job::DNFZinjection(gcs::inCollision msg){
    this->injection.index_from = msg.plan_index1;
    this->injection.index_to = msg.plan_index2;
    this->injection.start =msg.start;
    this->injection.to = msg.end;
    this->injection.valid_to = msg.valid_to;
    this->injection.stillValid = true;
}
void job::DNFZinjection(DNFZinject msg){
    this->injection = msg;
}
DNFZinject job::getDNFZinjection(){
    return this->injection;
}
void job::setWaitInAirTo(long waitTo){
    this->waitInAirTill = waitTo;
}
long job::getWaitTime(){
    return this->waitInAirTill;
}
void job::saveOldPlan(){
    if(NULL != this->getDrone()){
        this->TheOldPlan.plan = this->getDrone()->getPath();
        this->TheOldPlan.index = this->getDrone()->getMissionIndex();
    }else{
        std::cout <<"[Ground control]: "<< "can't save old plan no drone attached" << endl;
    }
}

oldPlan job::getOldPlan(){
    return this->TheOldPlan;
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

void drone::setVelocitySetPoint(double v){
    this->velocitySetPoint = v;
}

double drone::getVelocitySetPoint(){
    return this->velocitySetPoint;
}
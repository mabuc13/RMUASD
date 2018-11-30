
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


/*point collision[4];
    collision[0] = pointOfCollision( this->ourDrone.getCurHeading(),
                                        this->ourDrone.getPositionU(),
                                        this->otherDrone.getCurHeading(),
                                        this->otherDrone.getPositionU());

    collision[1] = pointOfCollision( this->ourDrone.getNextHeading(),
                                        this->ourDrone.getNextPositionU(),
                                        this->otherDrone.getCurHeading(),
                                        this->otherDrone.getPositionU());
    
    collision[2] = pointOfCollision( this->ourDrone.getCurHeading(),
                                        this->ourDrone.getPositionU(),
                                        this->otherDrone.getNextHeading(),
                                        this->otherDrone.getNextPositionU());
    
    collision[3] = pointOfCollision( this->ourDrone.getNextHeading(),
                                        this->ourDrone.getNextPositionU(),
                                        this->otherDrone.getNextHeading(),
                                        this->otherDrone.getNextPositionU());
    
    bool willBeThere1[4];
    bool willBeThere2[4];
    double tOurDrone[4];
    double tOtherDrone[4];
    // #################
    tOurDrone[0] = time2point(collision[0],this->ourDrone.getCurHeading(),
                                this->ourDrone.getEstimatedVelocity(),
                                this->ourDrone.getPositionU());
    tOurDrone[1] = time2point(collision[1],this->ourDrone.getNextHeading(),
                                this->ourDrone.getNextVelocity(),
                                this->ourDrone.getNextPositionU())
                                + this->ourDrone.getEtaNextWP();
                                
    tOurDrone[2] = time2point(collision[2],this->ourDrone.getCurHeading(),
                                this->ourDrone.getEstimatedVelocity(),
                                this->ourDrone.getPositionU());
    tOurDrone[3] = time2point(collision[0],this->ourDrone.getCurHeading(),
                                this->ourDrone.getEstimatedVelocity(),
                                this->ourDrone.getPositionU())
                                + this->ourDrone.getEtaNextWP();
    // #################
    tOtherDrone[0] = time2point(collision[0],this->otherDrone.getCurHeading(),
                                this->otherDrone.getEstimatedVelocity(),
                                this->otherDrone.getPositionU());
    tOtherDrone[1] = time2point(collision[1],this->otherDrone.getCurHeading(),
                                this->otherDrone.getEstimatedVelocity(),
                                this->otherDrone.getPositionU());
    tOtherDrone[2] = time2point(collision[2],this->otherDrone.getCurHeading(),
                                this->otherDrone.getEstimatedVelocity(),
                                this->otherDrone.getPositionU())
                                + this->otherDrone.getEtaNextWP();
    tOtherDrone[3] = time2point(collision[3],this->otherDrone.getCurHeading(),
                                this->otherDrone.getEstimatedVelocity(),
                                this->otherDrone.getPositionU())
                                + this->otherDrone.getEtaNextWP();

    willBeThere1[0] = tOurDrone[0]>0 && tOurDrone[0] < this->ourDrone.getEtaNextWP();
    willBeThere1[1] = this->ourDrone.getEtaNextWP()-tOurDrone[1]>0;      
    willBeThere1[2] = tOurDrone[2]>0 && tOurDrone[2] < this->ourDrone.getEtaNextWP(); 
    willBeThere1[3] = this->ourDrone.getEtaNextWP()-tOurDrone[3]>0; 

    willBeThere2[0] = tOtherDrone[0]>0 && tOtherDrone[0] < this->otherDrone.getEtaNextWP();   
    willBeThere2[1] = tOtherDrone[1]>0 && tOtherDrone[1] < this->otherDrone.getEtaNextWP(); 
    willBeThere2[2] = this->otherDrone.getEtaNextWP()-tOtherDrone[2]>0;    
    willBeThere2[3] = this->otherDrone.getEtaNextWP()-tOtherDrone[3]>0;  
                            
    if(tOurDrone < 0 || tOtherDrone <0 ){
        //TODO crashSite Behind drone
    }*/

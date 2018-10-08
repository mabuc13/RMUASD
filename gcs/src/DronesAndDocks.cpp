
#include "DronesAndDocks.hpp"

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

job::job(dock* station):status(1){
    goal = station;
    QuestGiver = station;
}
job::job(const job &ajob){
    this ->goal = ajob.goal;
    this-> QuestGiver = ajob.QuestGiver;
    this->status = ajob.status;
    this->setDrone(ajob.worker);
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
}

drone::drone(ID_t ID, gcs::GPS position):ID(ID),position(position),isFree(true){}
ID_t drone::getID(void){
    return this->ID;
}
gcs::GPS drone::getPosition(void){
    return this->position;
}
PATH_t drone::getPath(void){
    return this->thePath;
}
bool drone::isAvailable(void){
    return this->isFree;
}
job* drone::getJob(void){
    return currentJob;
}
void setAvailable(bool avail);
void setPath(PATH_t path);
void drone::setJob(job* aJob){
    if (this->currentJob != NULL){
        this->currentJob->setDrone(NULL);
    }
    this->currentJob = aJob;
    if(aJob != NULL){
        if(aJob->getDrone() != this){
            aJob->setDrone(this);
        }
    }
}
void drone::setPosition(gcs::GPS position){
    this->position = position;
}

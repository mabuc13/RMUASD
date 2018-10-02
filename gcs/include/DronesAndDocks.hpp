#ifndef  DRONES_AND_DOCKS_H
#define DRONES_AND_DOCKS_H 0

#include <dequeue>
#include <string>
#include <gcs/GPS>

#define ID_t unsigned int
#define PATH_t dequeue<GPS>

using namespace std;

class drone;

class dock{
public:
  dock(string name,double latitude, double longitude, double altitude, bool isLab);
  GPS getPosition(void);
  string getName(void);
  bool isLab(void);

private:
  GPS position;
  string name;
  bool isALab;

}

class job{
public:
  job(dock station);
  uint8 getStatus(void);
  drone* getDrone(void);
  dock* getQuestHandler(void);

  void setDrone(drone theDrone);
  void setGoal(dock goal);



  ~job();
private:



}

class drone{
public:
  drone(ID_t ID, GPS position);
  ID_t getID(void);
  GPS getPosition(void);
  PATH_t getPath(void);
  bool isAvailable(void);
  job getJob(void);


  void setAvailable(bool avail);
  void setPath(PATH_t path);
  void setJob(job* aJob);

private:
  job* currentJob;
  bool isFree;
  GPS position;
  ID_t ID;
  PATH_t thePath

}



#endif

#ifndef  DRONES_AND_DOCKS_H
#define DRONES_AND_DOCKS_H 0

#include <deque>
#include <vector>
#include <string>
#include <gcs/GPS.h>

#define ID_t unsigned int
#define PATH_t std::vector<gcs::GPS>
#define uint8 unsigned char

using namespace std;

class drone;

class dock{
public:
  dock(string name,double latitude, double longitude, double altitude, bool isLab);
  gcs::GPS getPosition(void);
  string getName(void);
  bool isLab(void);

private:
  gcs::GPS position;
  string name;
  bool isALab;

};

class job{
public:
  job(dock* station);
  job(const job &aJob);
  ~job();
  uint8 getStatus(void);
  drone* getDrone(void);
  dock* getQuestHandler(void);
  dock* getGoal(void);

  void setDrone(drone* theDrone);
  void setGoal(dock* goal);
  void setStatus(uint8 status);

  static const uint8 done = 6;
  static const uint8 ready4takeOff = 5;
  static const uint8 wait4pathplan = 4;
  static const uint8 onhold = 3;
  static const uint8 ongoing = 2;
  static const uint8 queued = 1;


private:
  dock* goal= NULL;
  dock* QuestGiver = (NULL);
  uint8 status;
  drone* worker = (NULL);




};

class drone{
public:
  drone(ID_t ID, gcs::GPS position);
  ID_t getID(void);
  gcs::GPS getPosition(void);
  std::vector<gcs::GPS> getPath(void);
  bool isAvailable(void);
  job* getJob(void);


  void setAvailable(bool avail);
  void setPath(const std::vector<gcs::GPS> &path);
  void setJob(job* aJob);
  void setPosition(gcs::GPS position);

private:
  job* currentJob = (NULL);
  bool isFree;
  gcs::GPS position;
  ID_t ID;
  std::vector<gcs::GPS> thePath;

};



#endif

#ifndef  DRONES_AND_DOCKS_H
#define DRONES_AND_DOCKS_H 0

#include <deque>
#include <vector>
#include <string>
#include <gcs/GPS.h>
#include <gcs/inCollision.h>

#include <UTM.hpp>
#define ID_t unsigned int
#define PATH_t std::vector<gcs::GPS>
#define uint8 unsigned char

using namespace std;

class drone;

struct DNFZinject{
  int index_from;
  int index_to;

  gcs::GPS start;
  gcs::GPS to;
  long time;
  size_t valid_to;
  int dnfz_id;
  bool stillValid;
};

struct oldPlan{
    std::vector<gcs::GPS> plan;
    int index;
};

struct direction{
  double north;
  double east;
};

struct UTM{
  double north;
  double east;
  double altitude;
  std::string zone;
  UTM& operator+=(const direction &b);
};


class dock{
public:
  dock(string name,double latitude, double longitude, double altitude, bool isLab, bool isEM=false);
  gcs::GPS getPosition(void);
  string getName(void);
  bool isLab(void);
  bool isEM(void);

private:
  gcs::GPS position;
  string name;
  bool isALab;
  bool isAEM;

};

class job{
public:
  job(dock* station);
  job(const job &aJob);
  ~job();
  uint8 getStatus(void);
  uint8 getNextStatus(void);
  drone* getDrone(void);
  dock* getQuestHandler(void);
  dock* getGoal(void);
  DNFZinject& getDNFZinjection();
  oldPlan getOldPlan();
  long getWaitTime();

  void setDrone(drone* theDrone);
  void setGoal(dock* goal);
  void setStatus(uint8 status);
  void setNextStatus(uint8 status);
  void setWaitInAirTo(long waitTo);
  void DNFZinjection(gcs::inCollision msg);
  void DNFZinjection(DNFZinject msg);
  bool& terminateJobOnLand();
  bool getTerminateJobOnLand();
  void saveOldPlan();

  static const uint8 ready4flightContinuation = 12;
  static const uint8 resumeFlight = 11;
  static const uint8 waitInAir = 10;
  static const uint8 rePathPlan = 9;
  static const uint8 preFlightCheack = 8;
  static const uint8 done = 7;
  static const uint8 ready4takeOff = 6;
  static const uint8 wait4pathplan = 5;
  static const uint8 onhold = 4;
  static const uint8 ongoing = 3;
  static const uint8 queued = 2;
  static const uint8 noMission = 1;
  static const uint8 notAssigned = 0;


private:
  dock* goal= NULL;
  dock* QuestGiver = (NULL);
  uint8 status;
  uint8 nextStatus;
  drone* worker = (NULL);

  DNFZinject injection;
  oldPlan TheOldPlan;
  long waitInAirTill;

  bool terminateOnLand;

};

class drone{
public:
  drone(ID_t ID, gcs::GPS position);
  ID_t getID(void);
  gcs::GPS getPosition(void);
  std::vector<gcs::GPS>& getPath(void);
  bool isAvailable(void);
  job* getJob(void);
  double getVelocity();
  double getVelocitySetPoint();
  size_t getMissionIndex();
  int getStatus();
  double& getGroundHeight();
  double& getFlightHeight();
  uint8& OS();
  gcs::GPS forwardPosition(double meters);



  void setAvailable(bool avail);
  void setPath(const std::vector<gcs::GPS> &path);
  void setJob(job* aJob);
  void setPosition(gcs::GPS position);
  void setVelocity(double v);
  void setVelocitySetPoint(double v);
  void setMissionIndex(size_t i);
  void setStatus(int status);
  void setHeading(double heading);

  static const uint8 normal_operation = 1;
  static const uint8 emergency_plan_with_fallback = 2;
  static const uint8 emergency = 3;

private:
  job* currentJob = (NULL);
  bool isFree;
  int status;
  gcs::GPS position;
  ID_t ID;
  std::vector<gcs::GPS> thePath;
  double velocity;
  double velocitySetPoint;
  size_t pathIndex;

  double groundHeight;
  double flightHeight;
  double heading;
  uint8 operationStatus;

};



#endif

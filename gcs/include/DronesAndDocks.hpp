#ifndef  DRONES_AND_DOCKS_H
#define DRONES_AND_DOCKS_H 0

#include <deque>
#include <vector>
#include <string>
#include <gcs/GPS.h>
#include <gcs/UTMDrone.h>

#include <UTM.hpp>
#define ID_t unsigned int
#define PATH_t std::vector<gcs::GPS>
#define uint8 unsigned char

using namespace std;

class drone;

struct UTM{
  double north;
  double east;
  double altitude;
  std::string zone;
};
struct direction{
  double north;
  double east;
};

gcs::GPS UTM2GPS(UTM coord);
UTM GPS2UTM(gcs::GPS coord);

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

  static const uint8 preFlightCheack = 8;
  static const uint8 done = 7;
  static const uint8 ready4takeOff = 6;
  static const uint8 wait4pathplan = 5;
  static const uint8 onhold = 4;
  static const uint8 ongoing = 3;
  static const uint8 queued = 2;
  static const uint8 noMission = 1;


private:
  dock* goal= NULL;
  dock* QuestGiver = (NULL);
  uint8 status;
  drone* worker = (NULL);




};

class simpleDrone{
public:
  simpleDrone();
  simpleDrone(ID_t ID,gcs::GPS cur_pos);
  simpleDrone(gcs::UTMDrone info);

  void update_values(gcs::UTMDrone info);
  gcs::GPS getPosition();
  UTM getPositionU();
  UTM getNextPositionU();
  direction getCurHeading();
  direction getNextHeading();
  double getCurVelocity();
  double getNextVelocity();
  ID_t getID();
  double getTime();
  double getEtaNextWP();
  uint8_t getPriority();
  double getBatterySOC();


private:

  direction getHeading(double heading);
  gcs::GPS next_wp;
  gcs::GPS cur_pos;

  float next_vel;
  float cur_vel;
  float cur_vel_est;
  deque<float> vel_list;
  float vel_acc;

  float next_heading;
  float cur_heading;

  int time;
  int gps_time;

  int battery_soc;

  int drone_priority;

  size_t ETA_next_WP;

  ID_t drone_id;

};

class drone{
public:
  drone(ID_t ID, gcs::GPS position);
  ID_t getID(void);
  gcs::GPS getPosition(void);
  std::vector<gcs::GPS> getPath(void);
  bool isAvailable(void);
  job* getJob(void);
  double getVelocity();
  size_t getMissionIndex();


  void setAvailable(bool avail);
  void setPath(const std::vector<gcs::GPS> &path);
  void setJob(job* aJob);
  void setPosition(gcs::GPS position);
  void setVelocity(double v);
  void setMissionIndex(size_t i);


private:
  job* currentJob = (NULL);
  bool isFree;
  gcs::GPS position;
  ID_t ID;
  std::vector<gcs::GPS> thePath;
  double velocity;
  size_t pathIndex;

};



#endif

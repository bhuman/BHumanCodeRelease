#pragma once

#include "Utils/bush/tools/ConfigMap.h"
#include <string>
#include <vector>

class Session;
class Team;

class Robot
{
  static std::map<std::string, Robot*> getRobotsByName();
public:
  std::string lan;
  std::string wlan;
  std::string name;

  std::string getSettingsString(const Team& team) const;
  std::string getBestIP() const;

  static std::vector<Robot> getRobots();
  static void initRobotsByName(std::map<std::string, Robot*> &robotsByName);

  friend class Session;
};

CONFIGMAP_STREAM_IN_DELCARE(Robot);
CONFIGMAP_STREAM_OUT_DELCARE(Robot);

ConfigMap& operator << (ConfigMap& cv, const Robot& robot);
const ConfigMap& operator >> (const ConfigMap& cv, Robot& robot);

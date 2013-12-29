#include "Controller/Platform/Directory.h"
#include "Platform/File.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Platform.h"
#include <iostream>

#if defined(LINUX) || defined(MACOSX)
#include <cstdlib>
#include <sys/types.h>
#include <cerrno>
#endif

std::string Robot::getSettingsString(const Team& team) const
{
  ConfigMap cm;
  cm["teamNumber"] << team.number;
  cm["teamPort"] << team.port;
  cm["teamColor"] << team.color;
  cm["playerNumber"] << team.getPlayerNumber(*this);
  cm["location"] << team.location;
  return cm.str();
}

std::string Robot::getBestIP() const
{
  return Session::getInstance().getBestIP(this);
}

std::vector<Robot> Robot::getRobots()
{
  std::vector<Robot> robots;
  std::string robotsDir = "Robots";
  Directory d;
  if(d.open(*File::getFullNames(robotsDir).begin() + "/*"))
  {
    std::string dir;
    bool isDir;
    while(d.read(dir, isDir))
    {
      if(isDir)
      {
        ConfigMap cm;
        std::string s = linuxToPlatformPath(dir + "/network.cfg");
        int status = cm.read(s);
        if(status > 0)
        {
          Robot r;
          cm >> r;
          robots.push_back(r);
        }
      }
    }
  }
  else
    perror(("Cannot open " + robotsDir).c_str());
  return robots;
}

void Robot::initRobotsByName(std::map<std::string, Robot*> &robotsByName)
{
  std::vector<Robot> robots = getRobots();
  for(size_t i = 0; i < robots.size(); ++i)
  {
    robotsByName[robots[i].name] = new Robot(robots[i]);
  }
}

CONFIGMAP_STREAM_IN(ConfigMap, Robot);
CONFIGMAP_STREAM_OUT(ConfigMap, Robot);

ConfigMap& operator << (ConfigMap& cv, const Robot& robot)
{
  cv["lan"] << robot.lan;
  cv["wlan"] << robot.wlan;
  cv["name"] << robot.name;
  return cv;
}

const ConfigMap& operator >> (const ConfigMap& cv, Robot& robot)
{
  cv["lan"] >> robot.lan;
  cv["wlan"] >> robot.wlan;
  cv["name"] >> robot.name;
  return cv;
}

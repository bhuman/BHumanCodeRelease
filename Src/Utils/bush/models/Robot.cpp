#include "Utils/bush/tools/Directory.h"
#include "Platform/File.h"
#include "Tools/Streams/InStreams.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Platform.h"
#include <iostream>

#if defined LINUX || defined MACOS
#include <cstdlib>
#include <sys/types.h>
#include <cerrno>
#endif

std::string Robot::getBestIP(const Context& context) const
{
  return Session::getInstance().getBestIP(context, this);
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
        InMapFile stream(linuxToPlatformPath(dir + "/network.cfg"));
        if(stream.exists())
        {
          Robot r;
          stream >> r;
          robots.push_back(r);
        }
      }
    }
  }
  else
    perror(("Cannot open " + robotsDir).c_str());
  return robots;
}

void Robot::initRobotsByName(std::map<std::string, Robot*>& robotsByName)
{
  std::vector<Robot> robots = getRobots();
  for(size_t i = 0; i < robots.size(); ++i)
  {
    robotsByName[robots[i].name] = new Robot(robots[i]);
  }
}

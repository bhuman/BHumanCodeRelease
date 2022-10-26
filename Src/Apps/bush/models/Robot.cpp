#include "Robot.h"
#include "Session.h"
#include "tools/Directory.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include "Streaming/InStreams.h"

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
  if(d.open(File::getFullNames(robotsDir).front() + "/*"))
  {
    std::string dir;
    bool isDir;
    while(d.read(dir, isDir))
    {
      if(isDir)
      {
        InMapFile stream(dir + "/network.cfg");
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

/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Settings.h"
#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif
#ifdef TARGET_ROBOT
#include "Platform/Nao/NaoBody.h"
#include <cstdio>
#endif
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Global.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Streams/StreamHandler.h"

STREAMABLE(Robots,
{
  STREAMABLE(RobotId,
  {,
    (std::string) name,
    (std::string) headId,
    (std::string) bodyId,
  });
  ,
  (std::vector<RobotId>) robotsIds,
});

bool Settings::recover = false;

Settings Settings::settings(true);
bool Settings::loaded = false;

Settings::Settings(bool master)
{
  ASSERT(master);
}

Settings::Settings()
{
  static_assert(TEAM_BLUE == blue && TEAM_RED == red && TEAM_YELLOW == yellow && TEAM_BLACK == black, "These macros and enums have to match!");
  if(!loaded)
  {
    VERIFY(settings.load());
    loaded = true;
  }
  *this = settings;

#ifdef TARGET_SIM
  if(SystemCall::getMode() == SystemCall::simulatedRobot)
  {
    int index = atoi(RoboCupCtrl::controller->getRobotName().c_str() + 5) - 1;
    teamNumber = index < 6 ? 1 : 2;
    teamPort = 10000 + teamNumber;
    teamColor = index < 6 ? blue : red;
    playerNumber = index % 6 + 1;
  }

  headName = bodyName = "Nao";

  ConsoleRoboCupCtrl* ctrl = dynamic_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller);
  if(ctrl)
  {
    std::string logFileName = ctrl->getLogFile();
    if(logFileName != "")
    {
      QRegExp re("([A-Za-z]*)_([A-Za-z]*)__", Qt::CaseSensitive, QRegExp::RegExp2);
      int pos = re.indexIn(logFileName.c_str());
      if(pos != -1)
      {
        headName = re.capturedTexts()[1].toUtf8().constData();
        bodyName = re.capturedTexts()[2].toUtf8().constData();
      }
      else
        headName = bodyName = "Default";
    }
  }

#endif

  isDropInGame = location.find("DropIn") != std::string::npos;
  isGoalkeeper = !isDropInGame && playerNumber == 1;
}

bool Settings::load()
{
  if(!Global::theStreamHandler)
  {
    static StreamHandler streamHandler;
    Global::theStreamHandler = &streamHandler;
  }

#ifdef TARGET_ROBOT
  headName = SystemCall::getHostName();

  std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if(!robotsStream.exists())
  {
    TRACE("Could not load robots.cfg");
    return false;
  }
  else
  {
    Robots robots;
    robotsStream >> robots;
    std::string bodyId = NaoBody().getBodyId();
    for(const Robots::RobotId& robot : robots.robotsIds)
    {
      if(robot.bodyId == bodyId)
      {
        bodyName = robot.name;
        break;
      }
    }
    if(bodyName.empty())
    {
      TRACE("Could not find bodyName in robots.cfg! BodyId: %s", bodyId.c_str());
      return false;
    }
  }
#else
  headName = "Nao";
  bodyName = "Nao";
#endif

  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
  {
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", headName.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  if(headName == bodyName)
    printf("Hi, I am %s.\n", headName.c_str());
  else
    printf("Hi, I am %s (using %ss Body).\n", headName.c_str(), bodyName.c_str());
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", getName(teamColor));
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
  printf("magicNumber %d\n", magicNumber);
#endif

  return true;
}

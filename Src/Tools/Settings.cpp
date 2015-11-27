/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif
#ifdef TARGET_ROBOT
#include "Platform/Linux/NaoBody.h"
#include <cstdio>
#endif
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Tools/Global.h"
#include "Tools/Streams/StreamHandler.h"

bool Settings::recover = false;

Settings Settings::settings(true);
bool Settings::loaded = false;

Settings::Settings(bool master) :
  isDropInGame(false),
  teamNumber(0),
  teamColor(blue),
  playerNumber(0),
  location("Default"),
  teamPort(0)
{
  ASSERT(master);
}

void Settings::init()
{
  ASSERT(TEAM_BLUE == blue && TEAM_RED == red && TEAM_YELLOW == yellow && TEAM_BLACK == black);
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

  robotName = "Nao";

  ConsoleRoboCupCtrl* ctrl = dynamic_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller);
  if(ctrl)
  {
    std::string logFileName = ctrl->getLogFile();
    if(logFileName != "")
    {
      QRegExp re("[0-9]_[A-Za-z]*_[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]_[0-9][0-9]-[0-9][0-9]-[0-9][0-9].log", Qt::CaseSensitive, QRegExp::RegExp2);
      int pos = re.indexIn(logFileName.c_str());
      if(pos != -1)
      {
        robotName = logFileName.substr(pos + 2);
        robotName = robotName.substr(0, robotName.find("_"));
      }
      else
        robotName = "Default";
    }
  }

  bodyName = robotName.c_str();

#endif

  highestValidPlayerNumber = 6;
  lowestValidPlayerNumber = 1;
  isDropInGame = location.find("DropIn") != std::string::npos;
  isCornerChallenge = location.find("CornerChallenge") != std::string::npos;
  isCarpetChallenge = location.find("CarpetChallenge") != std::string::npos;
  isRealBallChallenge = location.find("RealisticBallChallenge") != std::string::npos;
  isGoalkeeper = !isDropInGame && playerNumber == 1;
}

bool Settings::load()
{
#ifdef TARGET_ROBOT
  robotName = SystemCall::getHostName();
  bodyName = NaoBody().getName();
#else
  robotName = "Nao";
  bodyName = "Nao";
#endif

  if(!Global::theStreamHandler)
  {
    static StreamHandler streamHandler;
    Global::theStreamHandler = &streamHandler;
  }

  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
  {
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", robotName.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", getName(teamColor));
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
#endif

  return true;
}

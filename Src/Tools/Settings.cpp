/**
* @file Tools/Settings.cpp
* Implementation of a class that provides access to settings-specific configuration directories.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#ifdef TARGET_SIM
#include "Controller/RoboCupCtrl.h"
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
  ASSERT(TEAM_BLUE == blue && TEAM_RED == red);
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
    teamNumber = index < 5 ? 1 : 2;
    teamPort = 10000 + teamNumber;
    teamColor = index < 5 ? blue : red;
    playerNumber = (index % 5) + 1;
  }
#endif
}

bool Settings::load()
{
#ifdef TARGET_ROBOT
  robot = NaoBody().getName();
#else
  robot = "Nao";
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
    TRACE("Could not load settings for robot \"%s\" from settings.cfg", robot.c_str());
    return false;
  }

#ifdef TARGET_ROBOT
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", teamColor == TEAM_BLUE ? "blue" : "red");
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
#endif
  return true;
}

/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#include "Settings.h"
#ifdef TARGET_SIM
#include "Controller/ConsoleRoboCupCtrl.h"
#endif
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Representations/Communication/RoboCupGameControlData.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InStreams.h"

Settings Settings::settings(true);
std::vector<std::string> Settings::scenarios = {"", ""};
bool Settings::loaded = false;

Settings::Settings(bool master)
{
  ASSERT(master);
}

Settings::Settings()
{
  static_assert(TEAM_BLUE == blue && TEAM_RED == red && TEAM_YELLOW == yellow && TEAM_BLACK == black
                && TEAM_WHITE == white && TEAM_GREEN == green && TEAM_ORANGE == orange
                && TEAM_PURPLE == purple && TEAM_BROWN == brown && TEAM_GRAY == gray,
                "These macros and enums have to match!");
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
    teamColor = index < 6
                ? TeamColor(RoboCupCtrl::controller->gameController.teamInfos[0].teamColor)
                : TeamColor(RoboCupCtrl::controller->gameController.teamInfos[1].teamColor);
    playerNumber = index % 6 + 1;
    if(scenarios[teamNumber - 1] != "")
    {
      scenario = scenarios[teamNumber - 1];
    }
  }

  headName = bodyName = "Nao";

  ConsoleRoboCupCtrl* ctrl = dynamic_cast<ConsoleRoboCupCtrl*>(RoboCupCtrl::controller);
  if(ctrl)
  {
    std::string logFileName = ctrl->getLogFile();
    if(logFileName != "")
    {
      headName = bodyName = "Default";
      LoggingTools::parseName(logFileName, nullptr, &headName, &bodyName, &scenario, &location, nullptr, &playerNumber);
    }
  }
#endif
}

bool Settings::load()
{
  InMapFile stream("settings.cfg");
  if(stream.exists())
  {
    stream >> *this;
    return true;
  }
  else
  {
    OUTPUT_ERROR("Could not load settings for robot \"" << headName.c_str() << "\" from settings.cfg");
    return false;
  }
}

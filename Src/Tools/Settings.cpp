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
#include "Tools/Debugging/Debugging.h"
#include <cstdio>
#endif
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"
#include "Tools/Global.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InStreams.h"

STREAMABLE(Robots,
{
  STREAMABLE(RobotId,
  {,
    (std::string) name,
    (std::string) headId,
    (std::string) bodyId,
  }),

  (std::vector<RobotId>) robotsIds,
});

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
      QRegExp re1("([A-Za-z]*)_([A-Za-z]*)__", Qt::CaseSensitive, QRegExp::RegExp2);
      QRegExp re2("([A-Za-z]*)_([A-Za-z]*)_([A-Za-z0-9]*)_([A-Za-z0-9]*)__");
      QRegExp re3("_([0-9][0-9]*)(_\\([0-9][0-9]*\\)){0,1}\\.");

      int pos1 = re1.indexIn(logFileName.c_str());
      int pos2 = re2.indexIn(logFileName.c_str());
      int pos3 = re3.indexIn(logFileName.c_str());

      if(pos2 != -1)
      {
        headName = re2.capturedTexts()[1].toUtf8().constData();
        bodyName = re2.capturedTexts()[2].toUtf8().constData();
        scenario = re2.capturedTexts()[3].toUtf8().constData();
        location = re2.capturedTexts()[4].toUtf8().constData();
      }
      else if(pos1 != -1)
      {
        headName = re1.capturedTexts()[1].toUtf8().constData();
        bodyName = re1.capturedTexts()[2].toUtf8().constData();
      }
      else
        bodyName = headName = "Default";

      if(pos3 != -1)
        playerNumber = re3.capturedTexts()[1].toUtf8().constData()[0] - '0';
    }
  }
#endif
}

bool Settings::load()
{
#ifdef TARGET_ROBOT
  headName = SystemCall::getHostName();

  std::string bhdir = File::getBHDir();
  InMapFile robotsStream(bhdir + "/Config/Robots/robots.cfg");
  if(!robotsStream.exists())
  {
    OUTPUT_ERROR("Could not load robots.cfg");
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
      OUTPUT_ERROR("Could not find bodyName in robots.cfg! BodyId: " << bodyId.c_str());
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
    OUTPUT_ERROR("Could not load settings for robot \"" << headName.c_str() << "\" from settings.cfg");
    return false;
  }

#ifdef TARGET_ROBOT
  if(headName == bodyName)
    printf("Hi, I am %s.\n", headName.c_str());
  else
    printf("Hi, I am %s (using %ss Body).\n", headName.c_str(), bodyName.c_str());
  printf("teamNumber %d\n", teamNumber);
  printf("teamPort %d\n", teamPort);
  printf("teamColor %s\n", TypeRegistry::getEnumName(teamColor));
  printf("playerNumber %d\n", playerNumber);
  printf("location %s\n", location.c_str());
  printf("scenario %s\n", scenario.c_str());
  printf("magicNumber %d\n", magicNumber);
#endif

  return true;
}

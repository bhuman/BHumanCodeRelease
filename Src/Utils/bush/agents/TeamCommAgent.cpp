#include "Utils/bush/agents/TeamCommAgent.h"
#include "Utils/bush/bhwrapper/TeamCommWrapper.h"
#include "Utils/bush/bhwrapper/Framework.h"
#include "Utils/bush/tools/Sleeper.h"
#include "Utils/bush/tools/StringTools.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include <QtConcurrentRun>
#include <QThread>

class TeamCommMessageHandler : public MessageHandler
{
  TeamCommAgent* parent;
public:
  TeamCommMessageHandler(TeamCommAgent* parent) : parent(parent) {}

  virtual bool handleMessage(InMessage& message)
  {
    switch(message.getMessageID())
    {
    case idRobotHealth:
    {
      RobotHealth robotHealth;
      message.bin >> robotHealth;
      parent->setPower(robotHealth.robotName, robotHealth.batteryLevel);
      return true;
    }
    default:
      return false;
    }
  }

  friend class TeamCommAgent;
};

TeamCommAgent::TeamCommAgent(unsigned short port)
  : port(port),
    bhFramework(0),
    messageHandler(new TeamCommMessageHandler(this)),
    teamCommWrapper(0)
{
}

TeamCommAgent::~TeamCommAgent()
{
  Framework::destroy("bush.TeamComm." + toString(port));
  bhFramework = 0;
  delete teamCommWrapper;
  delete messageHandler;
}

void TeamCommAgent::run()
{
  std::string processName = "bush.TeamComm." + toString(port);
  bhFramework = Framework::getInstance(processName, 0, port);
  teamCommWrapper = new TeamCommWrapper(Framework::getInstance(processName)->teamOut,
                                        Framework::getInstance(processName)->settings.teamPort, messageHandler);
  while(isRunning())
  {
    teamCommWrapper->receive();
    if(!isRunning()) break;
    teamCommWrapper->send();
    if(!isRunning()) break;
    Sleeper::msleep(100);
  }
}

void TeamCommAgent::setPower(const std::string& robot, const Power& power)
{
  this->power[robot] = power;
  emit powerChanged(&this->power);
}

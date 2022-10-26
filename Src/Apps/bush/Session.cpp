#include "Platform/BHAssert.h"
#include "Session.h"
#include "agents/PingAgent.h"
#include "agents/StatusAgent.h"
#include "cmdlib/Context.h"
#include "models/Robot.h"
#include "models/Team.h"

Session& Session::getInstance()
{
  static Session theInstance;
  return theInstance;
}

void Session::initialize()
{
  for(const Robot& robot : Robot::getRobots())
    robotsByName[robot.name] = new Robot(robot);

  pingAgent = new PingAgent(robotsByName);
  statusAgent = new StatusAgent(pingAgent, robotsByName);
}

void Session::destroy()
{
  delete statusAgent;
  statusAgent = nullptr;

  delete pingAgent;
  pingAgent = nullptr;

  for(const auto& it : robotsByName)
    delete it.second;
  robotsByName.clear();
}

std::string Session::getBestIP(const Context& context, const Robot* robot)
{
  Team* team = context.getSelectedTeam();

  if(team->deployDevice == "Wi-Fi")
    return robot->wlan;
  else if(team->deployDevice == "Ethernet")
    return robot->lan;
  else if(team->deployDevice == "auto")
  {
    ENetwork best = pingAgent->getBestNetwork(robot);
    if(best == WLAN)
      return robot->wlan;
    else if(best == LAN)
      return robot->lan;
    return "";
  }

  FAIL("Unknown deploy device " << team->deployDevice << ".");
  return "";
}

bool Session::isReachable(const Context& context, const Robot* robot)
{
  Team* team = context.getSelectedTeam();

  if(team->deployDevice == "Wi-Fi")
    return pingAgent->getPing(WLAN, robot) < PingAgent::pingTimeout;
  else if(team->deployDevice == "Ethernet")
    return pingAgent->getPing(LAN, robot) < PingAgent::pingTimeout;
  else
    return pingAgent->getBestNetwork(robot) != NONE;
}

void Session::registerPingListener(QObject* qObject)
{
  if(pingAgent)
    QObject::connect(pingAgent, SIGNAL(pingChanged(ENetwork, const Robot*, double)),
                     qObject, SLOT(setPing(ENetwork, const Robot*, double)));
}

void Session::removePingListener(QObject* qObject)
{
  QObject::disconnect(pingAgent, SIGNAL(pingChanged(ENetwork, const Robot*, double)),
                      qObject, SLOT(setPing(ENetwork, const Robot*, double)));
}

void Session::registerStatusListener(QObject* qObject, Robot* robot)
{
  if(statusAgent)
  {
    QObject::connect(statusAgent, SIGNAL(statusChanged(const Robot*, const Status*)),
                     qObject, SLOT(setStatus(const Robot*, const Status*)));
    statusAgent->reset(robot);
  }
}

void Session::removeStatusListener(QObject* qObject, Robot* robot)
{
  QObject::disconnect(statusAgent, SIGNAL(statusChanged(const Robot*, const Status*)),
                      qObject, SLOT(setStatus(const Robot*, const Status*)));
  statusAgent->reset(robot);
}

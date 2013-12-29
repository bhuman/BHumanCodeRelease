#include "Utils/bush/Session.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/agents/TeamCommAgent.h"
#include "Utils/bush/agents/RemoteRobotAgent.h"
#include "Utils/bush/cmdlib/IConsole.h"
#include <iostream>

Session::Session()
  : console(0),
    logLevel(ALL),
    pingAgent(0),
    teamCommAgents(),
    remoteRobotAgent(0),
    robotsByName()
{
}

Session::~Session()
{
}

std::string Session::getBestIP(const Robot* robot)
{
  std::string ip;
  ENetwork best = getBestNetwork(robot);
  if(best == WLAN)
    ip = robot->wlan;
  else if(best == LAN)
    ip = robot->lan;

  return ip;
}

Session& Session::getInstance()
{
  static Session theInstance;
  return theInstance;
}

void Session::registerConsole(IConsole* console)
{
  this->console = console;
  log(TRACE, "Session: Console registered.");
}

IConsole* Session::getConsole()
{
  return console;
}

void Session::log(LogLevel logLevel, const std::string& error)
{
  if(logLevel >= this->logLevel)
  {
    if(console)
      console->errorLine(error);
    else
      std::cerr << error << std::endl;
  }
}

bool Session::isReachable(const Robot* robot)
{
  return  getBestNetwork(robot) != NONE;
}

ENetwork Session::getBestNetwork(const Robot* robot)
{
  return pingAgent->getBestNetwork(robot);
}

void Session::registerPingListener(QObject* qObject)
{
  if(pingAgent)
  {
    log(TRACE, "Session: Registered ping listener.");
    QObject::connect(pingAgent, SIGNAL(pingChanged(ENetwork, std::map<std::string, double>*)),
                     qObject, SLOT(setPings(ENetwork, std::map<std::string, double>*)));
  }
  else
  {
    log(WARN, "Session: Could not register ping listener. No pingAgent initialized.");
  }
}

void Session::removePingListener(QObject* qObject)
{
  log(TRACE, "Session: Removed ping listener.");
  QObject::disconnect(pingAgent, SIGNAL(pingChanged(ENetwork, std::map<std::string, double>*)),
                      qObject, SLOT(setPings(ENetwork, std::map<std::string, double>*)));
}

void Session::registerPowerListener(QObject* qObject)
{
  if(teamCommAgents.empty())
  {
    log(WARN, "Session: Could not register power listener. No teamCommAgents initialized.");
  }
  else
  {
    log(TRACE, "Session: Registered power listener.");
    for(std::vector<TeamCommAgent*>::iterator it = teamCommAgents.begin();
        it != teamCommAgents.end(); it++)
      QObject::connect(*it, SIGNAL(powerChanged(std::map<std::string, Power>*)),
                       qObject, SLOT(setPower(std::map<std::string, Power>*)));
  }
}

void Session::removePowerListener(QObject* qObject)
{
  log(TRACE, "Session: Removed power listener.");
  for(std::vector<TeamCommAgent*>::iterator it = teamCommAgents.begin();
      it != teamCommAgents.end(); it++)
    QObject::disconnect(*it, SIGNAL(powerChanged(std::map<std::string, Power>*)),
                        qObject, SLOT(setPower(std::map<std::string, Power>*)));
}

std::vector<std::string> Session::sendDebugRequest(const Robot* robot, const std::string& command)
{
  return remoteRobotAgent->sendDebugRequestToRobot(robot, command);
}

void Session::addTeamCommAgent(Team* team)
{
  TeamCommAgent* tca = new TeamCommAgent(team->port);
  teamCommAgents.push_back(tca);
  tca->start(tca, &TeamCommAgent::run);
}

void Session::removeTeamCommAgent(Team* team)
{
  for(size_t i = 0; i < teamCommAgents.size(); ++i)
  {
    TeamCommAgent* tca = teamCommAgents[i];
    if(tca->getPort() == team->port)
    {
      teamCommAgents.erase(teamCommAgents.begin() + i);
      tca->deleteLater();
    }
  }
}

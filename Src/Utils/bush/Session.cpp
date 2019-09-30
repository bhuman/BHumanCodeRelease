#include "Platform/BHAssert.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/models/Team.h"
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/agents/StatusAgent.h"
#include "Utils/bush/cmdlib/IConsole.h"
#include "Utils/bush/cmdlib/Context.h"
#include <iostream>

Session::Session()
  : console(0),
    logLevel(ALL),
    pingAgent(0),
    statusAgent(0)
{
}

std::string Session::getBestIP(const Context& context, const Robot* robot)
{
  std::string ip;
  Team* team = context.getSelectedTeam();

  if(team->deployDevice == "Wi-Fi")
    ip = robot->wlan;
  else if(team->deployDevice == "Ethernet")
    ip = robot->lan;
  else if(team->deployDevice == "auto")
  {
    ENetwork best = getBestNetwork(robot);
    if(best == WLAN)
      ip = robot->wlan;
    else if(best == LAN)
      ip = robot->lan;
  }
  else
    FAIL("Unknown deploy device " << team->deployDevice << ".");

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

bool Session::isReachable(const Context& context, const Robot* robot)
{
  Team* team = context.getSelectedTeam();

  if(team->deployDevice == "Wi-Fi")
    return pingAgent->getWLanPing(robot) < 2000.0;
  else if(team->deployDevice == "Ethernet")
    return pingAgent->getLanPing(robot) < 2000.0;
  else
    return getBestNetwork(robot) != NONE;
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

void Session::registerStatusListener(QObject* qObject, Robot* robot)
{
  if(statusAgent)
  {
    log(TRACE, "Session: Registered power listener.");
    QObject::connect(statusAgent, SIGNAL(powerChanged(std::map<std::string, Power>*)),
                     qObject, SLOT(setPower(std::map<std::string, Power>*)));
    QObject::connect(statusAgent, SIGNAL(logsChanged(std::map<std::string, int>*)),
                     qObject, SLOT(setLogs(std::map<std::string, int>*)));
    statusAgent->reset(robot);
  }
  else
  {
    log(WARN, "Session: Could not register power listener. No powerAgent initialized.");
  }
}

void Session::removeStatusListener(QObject* qObject, Robot* robot)
{
  log(TRACE, "Session: Removed power listener.");
  QObject::disconnect(statusAgent, SIGNAL(powerChanged(std::map<std::string, Power>*)),
                      qObject, SLOT(setPower(std::map<std::string, Power>*)));
  QObject::disconnect(statusAgent, SIGNAL(logsChanged(std::map<std::string, int>*)),
                      qObject, SLOT(setLogs(std::map<std::string, int>*)));
  statusAgent->reset(robot);
}

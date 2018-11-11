#include "Utils/bush/agents/StatusAgent.h"
#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/Sleeper.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/models/Robot.h"
#include "Platform/Time.h"

#define UPDATE_TIME 5000

StatusAgent::StatusAgent(PingAgent* pingAgent) :
  pingAgent(pingAgent)
{}

StatusAgent::~StatusAgent()
{
  for(auto it = processes.cbegin(), end = processes.cend(); it != end; ++it)
    if(it->second)
      delete it->second;
}

void StatusAgent::initialize(std::map<std::string, Robot*>& robotsByName)
{
  Session::getInstance().registerPingListener(this);
  for(auto it = robotsByName.cbegin(), end = robotsByName.cend(); it != end; ++it)
  {
    power[it->first] = Power();
    logs[it->first] = 0;
    timeOfLastUpdate[it->first] = -UPDATE_TIME;
    processes[it->first] = new QProcess(this);
    connect(processes[it->first], SIGNAL(readyReadStandardOutput()), this, SLOT(statusReadable()));
  }
  emit powerChanged(&this->power);
  emit logsChanged(&this->logs);
}

void StatusAgent::setPings(ENetwork, std::map<std::string, double>*)
{
  const unsigned currentTime = Time::getRealSystemTime();

  for(auto it = Session::getInstance().robotsByName.cbegin(), end = Session::getInstance().robotsByName.cend(); it != end; ++it)
  {
    if(currentTime - timeOfLastUpdate[it->first] > UPDATE_TIME && pingAgent->getBestNetwork(it->second) != ENetwork::NONE)
    {
      const std::string ip = pingAgent->getBestNetwork(it->second) == ENetwork::LAN ? it->second->lan : it->second->wlan;
      const std::string cmd = "bash -l -c \\'battery.py " + it->second->name + "\\' | grep " + it->second->name;

      processes[it->first]->start(fromString(remoteCommandForQProcess(cmd, ip)));

      timeOfLastUpdate[it->first] = currentTime;
    }
    if(currentTime - pingAgent->getLastConnectionTime(it->second) > DISCONNECTED_TIME)
    {
      reset(it->second);
    }
  }
}

void StatusAgent::statusReadable()
{
  QProcess* process = dynamic_cast<QProcess*>(sender());

  const QByteArray data = process->readAllStandardOutput();
  const QString output(data);

  const QStringList s = output.split(' ');
  const std::string name = toString(s[0]);

  power[name].value = static_cast<int>(s[1].toFloat() * 100.f);
  power[name].batteryCharging = s.size() > 2
                                ? static_cast<short>(s[2].trimmed().toFloat()) & 0b10000000
                                : false;

  logs[name] = s.size() > 3
               ? s[3].toInt()
               : 0;

  emit powerChanged(&this->power);
  emit logsChanged(&this->logs);
}

void StatusAgent::reset(Robot* robot)
{
  if(robot)
  {
    power[robot->name].value = 101; // Invalid Value
    power[robot->name].batteryCharging = false;
    timeOfLastUpdate[robot->name] = -UPDATE_TIME;

    logs[robot->name] = false;

    emit powerChanged(&this->power);
    emit logsChanged(&this->logs);
  }
}

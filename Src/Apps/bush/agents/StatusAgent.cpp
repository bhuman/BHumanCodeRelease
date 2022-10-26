#include "StatusAgent.h"
#include "agents/PingAgent.h"
#include "models/Robot.h"
#include "Session.h"
#include "tools/ShellTools.h"
#include "Platform/Time.h"
#include <QString>

StatusAgent::StatusAgent(PingAgent* pingAgent, const std::map<std::string, Robot*>& robotsByName) :
  pingAgent(pingAgent)
{
  connect(pingAgent, &PingAgent::pingChanged, this, &StatusAgent::setPing);
  for(auto it = robotsByName.cbegin(), end = robotsByName.cend(); it != end; ++it)
  {
    reset(it->second);

    processes[it->second] = new QProcess(this);
    connect(processes[it->second], &QProcess::readyReadStandardOutput, this, [this, robot = it->second]{ statusReadable(robot); });
  }
}

StatusAgent::~StatusAgent()
{
  for(auto it = processes.cbegin(), end = processes.cend(); it != end; ++it)
    it->second->close();
}

void StatusAgent::setPing(ENetwork, const Robot* robot, double)
{
  const unsigned currentTime = Time::getRealSystemTime();

  if(currentTime - timeOfLastUpdate[robot] > updateTime && pingAgent->getBestNetwork(robot) != ENetwork::NONE
     && processes[robot]->state() == QProcess::NotRunning)
  {
    const std::string ip = pingAgent->getBestNetwork(robot) == ENetwork::LAN ? robot->lan : robot->wlan;
    const std::string cmd = "( cat /var/volatile/tmp/batteryLevel.txt; "
                              "echo -n ' '; "
                              "ls /home/nao/logs | wc -l; "
                              "echo -n ' '; "
                              "[ -e /home/nao/bhdump.log ] && echo yes || echo no ) | tr -d '\\n'";

    processes[robot]->startCommand(QString::fromStdString(remoteCommandForQProcess(cmd, ip)));

    timeOfLastUpdate[robot] = currentTime;
  }

  if(currentTime - pingAgent->getLastConnectionTime(robot) > PingAgent::disconnectedTimeout)
    reset(robot);
}

void StatusAgent::statusReadable(const Robot* robot)
{
  QProcess* process = dynamic_cast<QProcess*>(sender());

  const QByteArray data = process->readAllStandardOutput();
  const QString output(data);

  const QStringList s = output.trimmed().split(' ');
  if(s.size() != 4)
  {
    reset(robot);
    return;
  }

  status[robot].batteryLevel = static_cast<int>(s[0].toFloat() * 100.f);
  status[robot].batteryCharging = (static_cast<short>(s[1].trimmed().toFloat()) & 0b10000000) != 0;
  status[robot].logs = s[2].toInt();
  status[robot].hasDump = s[3] == "yes";

  emit statusChanged(robot, &status[robot]);
}

void StatusAgent::reset(const Robot* robot)
{
  if(robot)
  {
    status[robot] = Status();
    timeOfLastUpdate[robot] = -updateTime;

    emit statusChanged(robot, &status[robot]);
  }
}

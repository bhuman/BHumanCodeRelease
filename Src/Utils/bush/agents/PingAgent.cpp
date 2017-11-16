#include "Utils/bush/agents/PingAgent.h"
#include "Utils/bush/models/Robot.h"
#include "Utils/bush/Session.h"
#include "Utils/bush/tools/StringTools.h"
#include "Platform/Time.h"
#include <QProcess>
#include <QByteArray>
#include <QString>
#include <QStringList>
#include <QRegExp>
#include <iostream>

PingAgent::~PingAgent()
{
  cleanUp();
}

void PingAgent::cleanUp()
{
  for(size_t n = 0; n < ENetworkSize; ++n)
  {
    for(int i = 0; i < pingProcesses[n].size(); i++)
    {
      pingProcesses[n][i]->close();
      delete pingProcesses[n][i];
      pingProcesses[n][i] = 0;
    }
    pingProcesses[n].clear();
    pings[n].clear();
  }
  pingProcesses.clear();
  robots.clear();
  pings.clear();
}

void PingAgent::initializeProcesses(std::map<std::string, Robot*>& robotsByName)
{
  for(size_t n = 0; n < ENetworkSize; ++n)
  {
    pingProcesses[n] = QList<QProcess*>();
    pings[n] = std::map<std::string, double>();
    int i = 0;
    for(std::map<std::string, Robot*>::iterator it = robotsByName.begin();
        it != robotsByName.end(); it++, i++)
    {
      pingProcesses[n] << new QProcess(this);

      robots[pingProcesses[n][i]] = it->second;
      pings[n][it->second->name] = 2000.0;
      lastConnectionTime[it->second] = -DISCONNECTED_TIME;

#ifdef WINDOWS
      std::string pingParameters = "-t ";
#else
      std::string pingParameters;
#endif

      switch(n)
      {
        case WLAN:
        {
          QObject::connect(pingProcesses[n][i], SIGNAL(readyReadStandardOutput()), this, SLOT(pingReadableWLAN()));
          QString command = fromString("ping " + pingParameters + it->second->wlan);
          pingProcesses[n][i]->start(command);
          Session::getInstance().log(TRACE, "PingAgent: Started ping process for " + it->second->wlan);
        }
        break;
        case LAN:
        {
          QObject::connect(pingProcesses[n][i], SIGNAL(readyReadStandardOutput()), this, SLOT(pingReadableLAN()));
          QString command = fromString("ping " + pingParameters + it->second->lan);
          pingProcesses[n][i]->start(command);
          Session::getInstance().log(TRACE, "PingAgent: Started ping process for " + it->second->lan);
        }
        break;
        default:
          Session::getInstance().log(TRACE, "PingAgent: Unknown network type.");
      }
    }
  }
}

ENetwork PingAgent::getBestNetwork(const Robot* robot)
{
  double wlanPing = pings[WLAN][robot->name];
  double lanPing  = pings[LAN][robot->name];
  if(wlanPing == 2000.0 && lanPing == 2000.0)
    return NONE;
  else if(wlanPing < lanPing)
    return WLAN;
  else
    return LAN;
}

double PingAgent::getWLanPing(const Robot* robot)
{
  return pings[WLAN][robot->name];
}

double PingAgent::getLanPing(const Robot* robot)
{
  return pings[LAN][robot->name];
}

int PingAgent::getLastConnectionTime(Robot* robot)
{
  return lastConnectionTime[robot];
}

void PingAgent::robotsChanged()
{
  cleanUp();
  initializeProcesses(Session::getInstance().robotsByName);
}

void PingAgent::pingReadableWLAN()
{
  QProcess* process = dynamic_cast<QProcess*>(sender());
  updatePing(WLAN, process);
}

void PingAgent::pingReadableLAN()
{
  QProcess* process = dynamic_cast<QProcess*>(sender());
  updatePing(LAN, process);
}

void PingAgent::updatePing(ENetwork network, QProcess* process)
{
  std::map<std::string, double>& networkMap = network == WLAN ? pings[WLAN] : pings[LAN];
  std::map<std::string, double>& otherNetworkMap = network == WLAN ? pings[LAN] : pings[WLAN];
  QByteArray data = process->readAllStandardOutput();
  QString pingOutput(data);

  QStringList splittedOutput = pingOutput.split(" ", QString::SkipEmptyParts);
  QStringList filteredSplittedOutput = splittedOutput.filter(QRegExp("((Zeit|Time|time)[<=]\\d+ms|time=\\d+(\\.\\d+)?)"));

  const unsigned currentTime = Time::getRealSystemTime();

  if(filteredSplittedOutput.empty())
  {
    networkMap[robots[process]->name] = 2000.0;
  }
  else
  {
    QString timeString = filteredSplittedOutput[0];
    QStringList keyAndValue = timeString.split(QRegExp("[<=]"), QString::SkipEmptyParts);
    if(keyAndValue.size() < 2)
    {
      networkMap[robots[process]->name] = 2000.0;
      lastConnectionTime[robots[process]] = currentTime;
    }
    else
    {
      QString value = keyAndValue[1].replace("ms", "");
      lastConnectionTime[robots[process]] = currentTime;
      networkMap[robots[process]->name] = value.toDouble();
      otherNetworkMap[robots[process]->name] += 200.0; // hack!
      if(otherNetworkMap[robots[process]->name] > 2000.0)
        otherNetworkMap[robots[process]->name] = 2000.0;
    }
  }

  emit pingChanged(network, &networkMap);
}

#include "PingAgent.h"
#include "models/Robot.h"
#include "Session.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include <QByteArray>
#include <QProcess>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <QString>
#include <QStringList>

PingAgent::PingAgent(const std::map<std::string, Robot*>& robotsByName)
{
  QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
  env.insert("LC_ALL", "C");
  for(ENetwork n : {WLAN, LAN})
  {
    for(auto it = robotsByName.begin(); it != robotsByName.end(); it++)
    {
      pingProcesses[n] << new QProcess(this);
      pingProcesses[n].back()->setProcessEnvironment(env);

      pings[n][it->second] = pingTimeout;
      lastConnectionTime[it->second] = -disconnectedTimeout;

      QStringList pingParameters;
#ifdef WINDOWS
      pingParameters << "-t";
#endif

      QObject::connect(pingProcesses[n].back(), &QProcess::readyReadStandardOutput, this, [this, n, robot = it->second]{ updatePing(n, robot); });
      switch(n)
      {
        case WLAN:
          pingParameters << QString::fromStdString(it->second->wlan);
          break;
        case LAN:
          pingParameters << QString::fromStdString(it->second->lan);
          break;
        default:
          FAIL("Unknown network type.");
      }
      pingProcesses[n].back()->start("ping", pingParameters);
    }
  }
}

PingAgent::~PingAgent()
{
  for(const auto& processesByNetwork : pingProcesses)
    for(QProcess* process : processesByNetwork.second)
      process->close();
}

ENetwork PingAgent::getBestNetwork(const Robot* robot)
{
  const double wlanPing = pings[WLAN][robot];
  const double lanPing = pings[LAN][robot];
  if(wlanPing == pingTimeout && lanPing == pingTimeout)
    return NONE;
  else if(wlanPing < lanPing)
    return WLAN;
  else
    return LAN;
}

double PingAgent::getPing(ENetwork network, const Robot* robot)
{
  return pings[network][robot];
}

int PingAgent::getLastConnectionTime(const Robot* robot) const
{
  if(const auto it = lastConnectionTime.find(robot); it != lastConnectionTime.end())
    return it->second;
  return -disconnectedTimeout;
}

void PingAgent::updatePing(ENetwork network, const Robot* robot)
{
  QProcess* process = dynamic_cast<QProcess*>(sender());
  std::map<const Robot*, double>& networkMap = pings[network];
  std::map<const Robot*, double>& otherNetworkMap = network == WLAN ? pings[LAN] : pings[WLAN];
  const QByteArray data = process->readAllStandardOutput();
  const QString pingOutput(data);

  const QStringList splittedOutput = pingOutput.split(" ", Qt::SkipEmptyParts);
  const QStringList filteredSplittedOutput = splittedOutput.filter(QRegularExpression("((Zeit|Time|time)[<=]\\d+ms|time=\\d+(\\.\\d+)?)"));
  const unsigned currentTime = Time::getRealSystemTime();

  if(filteredSplittedOutput.empty())
  {
    networkMap[robot] = pingTimeout;
  }
  else
  {
    const QString timeString = filteredSplittedOutput[0];
    QStringList keyAndValue = timeString.split(QRegularExpression("[<=]"), Qt::SkipEmptyParts);
    if(keyAndValue.size() < 2)
    {
      networkMap[robot] = pingTimeout;
      lastConnectionTime[robot] = currentTime;
    }
    else
    {
      const QString value = keyAndValue[1].replace("ms", "");
      lastConnectionTime[robot] = currentTime;
      networkMap[robot] = value.toDouble();
      otherNetworkMap[robot] += pingTimeout * 0.1; // hack!
      if(otherNetworkMap[robot] > pingTimeout)
        otherNetworkMap[robot] = pingTimeout;
    }
  }

  emit pingChanged(network, robot, networkMap[robot]);
}

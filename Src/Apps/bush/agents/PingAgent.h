#pragma once

#include "Session.h"
#include <QObject>
#include <QList>
#include <map>

struct Robot;
class QProcess;

class PingAgent : public QObject
{
  Q_OBJECT

  std::map<ENetwork, QList<QProcess*>> pingProcesses;
  std::map<ENetwork, std::map<const Robot*, double>> pings;
  std::map<const Robot*, int> lastConnectionTime;

public:
  PingAgent(const std::map<std::string, Robot*>& robotsByName);
  ~PingAgent();
  ENetwork getBestNetwork(const Robot* robot);
  double getPing(ENetwork network, const Robot* robot);
  int getLastConnectionTime(const Robot* robot) const;

  static constexpr double pingTimeout = 2000.0;
  static constexpr int disconnectedTimeout = 5000;

private slots:
  void updatePing(ENetwork network, const Robot* robot);

signals:
  void pingChanged(ENetwork network, const Robot* robot, double ping);
};

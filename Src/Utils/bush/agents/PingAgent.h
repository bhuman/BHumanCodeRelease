#pragma once

#include "Utils/bush/Session.h"
#include <QObject>
#include <QList>
#include <map>

class Robot;
class QProcess;

class PingAgent : public QObject
{
  Q_OBJECT

  std::map<size_t, QList<QProcess*> > pingProcesses;
  std::map<QProcess*, Robot*> robots;
  std::map<size_t, std::map<std::string, double> > pings;

public:
  PingAgent();
  ~PingAgent();
  void cleanUp();
  void initializeProcesses(std::map<std::string, Robot*>& robotsByName);
  ENetwork getBestNetwork(const Robot* robot);
  void updatePing(ENetwork network, QProcess* process);

public slots:
  void robotsChanged();
  void pingReadableWLAN();
  void pingReadableLAN();

signals:
  void pingChanged(ENetwork network, std::map<std::string, double> *pings);
};

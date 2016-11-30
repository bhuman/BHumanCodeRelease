#pragma once

#include "Utils/bush/Session.h"
#include <QObject>
#include <QList>
#include <map>

#define DISCONNECTED_TIME 5000

struct Robot;
class QProcess;

class PingAgent : public QObject
{
  Q_OBJECT

  std::map<size_t, QList<QProcess*>> pingProcesses;
  std::map<QProcess*, Robot*> robots;
  std::map<size_t, std::map<std::string, double>> pings;
  std::map<Robot*, int> lastConnectionTime;

public:
  ~PingAgent();
  void cleanUp();
  void initializeProcesses(std::map<std::string, Robot*>& robotsByName);
  ENetwork getBestNetwork(const Robot* robot);
  double getWLanPing(const Robot* robot);
  double getLanPing(const Robot* robot);
  int getLastConnectionTime(Robot* robot);
  void updatePing(ENetwork network, QProcess* process);

public slots:
  void robotsChanged();
  void pingReadableWLAN();
  void pingReadableLAN();

signals:
  void pingChanged(ENetwork network, std::map<std::string, double>* pings);
};

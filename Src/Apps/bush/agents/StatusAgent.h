#pragma once

#include "Session.h"
#include "models/Status.h"
#include <QObject>
#include <QProcess>
#include <map>
#include <string>

class PingAgent;
struct Robot;

class StatusAgent : public QObject
{
  Q_OBJECT

  PingAgent* pingAgent;
  std::map<const Robot*, Status> status;
  std::map<const Robot*, int> timeOfLastUpdate;
  std::map<const Robot*, QProcess*> processes;

  static constexpr int updateTime = 5000;

public:
  StatusAgent(PingAgent* pingAgent, const std::map<std::string, Robot*>& robotsByName);
  ~StatusAgent();

  void reset(const Robot* robot);

private slots:
  void setPing(ENetwork, const Robot*, double);
  void statusReadable(const Robot*);

signals:
  void statusChanged(const Robot*, const Status*);
};

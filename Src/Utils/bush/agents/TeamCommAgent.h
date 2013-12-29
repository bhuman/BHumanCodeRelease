#pragma once

#include "Utils/bush/models/Power.h"
#include <map>
#include <string>
#include <QObject>
#include "Platform/Thread.h"

class Framework;
class MessageHandler;
class TeamCommMessageHandler;
class TeamCommWrapper;

class TeamCommAgent : public QObject, public Thread<TeamCommAgent>
{
  Q_OBJECT

  friend class TeamCommMessageHandler;

  unsigned short port;
  Framework* bhFramework;
  MessageHandler* messageHandler;
  TeamCommWrapper* teamCommWrapper;
  std::map<std::string, Power> power;

  void setPower(const std::string& robot, const Power& power);
public:
  TeamCommAgent(unsigned short port);
  ~TeamCommAgent();

  void run();

  inline unsigned short getPort() { return port; }

signals:
  void powerChanged(std::map<std::string, Power>*);
};

#pragma once

#include <map>
#include <string>
#include <vector>
#include <QObject>

class Context;
class PingAgent;
class StatusAgent;
struct Robot;
class Team;
class CommandContext;

enum ENetwork
{
  WLAN = 0,
  LAN,
  ENetworkSize,
  NONE // This is invalid (do not create an entry in the maps for it)
};

class Session : public QObject
{
  Q_OBJECT

  friend class Initializer;

  PingAgent* pingAgent = nullptr;
  StatusAgent* statusAgent = nullptr;

  Session() = default;

public:
  std::map<std::string, Robot*> robotsByName;

  static Session& getInstance();

  void initialize();

  void destroy();

  std::string getBestIP(const Context& context, const Robot* robot);
  bool isReachable(const Context& context, const Robot* robot);

  void registerPingListener(QObject* qObject);
  void removePingListener(QObject* qObject);

  void registerStatusListener(QObject* qObject, Robot* robot = nullptr);
  void removeStatusListener(QObject* qObject, Robot* robot = nullptr);
};

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
class IConsole;
class CommandContext;

enum ENetwork
{
  WLAN = 0,
  LAN,
  ENetworkSize,
  NONE // This is invalid (do not create an entry in the maps for it)
};

enum LogLevel
{
  ALL = 0,
  TRACE = 0,
  WARN = 1,
  CRITICAL = 2,
  FATAL = 3,
  OFF = 4
};

class Session : public QObject
{
  Q_OBJECT

  friend class Initializer;

  IConsole* console;
  LogLevel logLevel;

  PingAgent* pingAgent;
  StatusAgent* statusAgent;

  Session();

  ENetwork getBestNetwork(const Robot* robot);

public:
  std::map<std::string, Robot*> robotsByName;

  static Session& getInstance();

  void registerConsole(IConsole* console);
  IConsole* getConsole();

  void log(LogLevel logLevel, const std::string& message);

  std::string getBestIP(const Context& context, const Robot* robot);
  bool isReachable(const Context& context, const Robot* robot);

  void registerPingListener(QObject* qObject);
  void removePingListener(QObject* qObject);

  void registerStatusListener(QObject* qObject, Robot* robot = nullptr);
  void removeStatusListener(QObject* qObject, Robot* robot = nullptr);

  std::vector<std::string> sendDebugRequest(const Context& context, const Robot* robot, const std::string& command);

signals:
  void robotsChanged();
};

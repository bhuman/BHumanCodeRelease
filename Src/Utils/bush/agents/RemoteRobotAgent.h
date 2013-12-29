#pragma once

#include <string>
#include <vector>

class Robot;

class RemoteRobotAgent
{
  friend class RemoteMessageHandler;

  std::vector<std::string> answers;

  void appendAnswer(const std::string& answer);
public:
  // TODO more general interface for mr, dr, get, set etc.
  std::vector<std::string> sendDebugRequestToRobot(const Robot* robots, const std::string& command);
};

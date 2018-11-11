#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class SCPCmd : public RobotCommand
{
  class SCPTask : public RobotTask
  {
    std::string command;
  public:
    SCPTask(Context& context, Robot* robot, const std::string& command);
    bool execute();
  };

  bool fromRobot;
  std::string fromFile;
  std::string toFile;

  SCPCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

public:
  static SCPCmd theSCPCmd;
};

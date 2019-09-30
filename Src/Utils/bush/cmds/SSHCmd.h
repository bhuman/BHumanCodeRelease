#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class SSHCmd : public RobotCommand
{
  class SSHTask : public RobotTask
  {
    std::string command;
  public:
    SSHTask(Context& context, Robot* robot, const std::string& command);
    bool execute();
  };

  std::string command;

  SSHCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

public:
  static SSHCmd theSSHCmd;
};

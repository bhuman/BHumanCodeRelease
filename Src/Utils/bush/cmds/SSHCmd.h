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
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, Robot& robot);
public:
  static SSHCmd theSSHCmd;
};

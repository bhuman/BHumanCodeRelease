#pragma once

#include "cmdlib/RobotCommand.h"

class SSHCmd : public RobotCommand<SSHCmd>
{
public:
  SSHCmd();

private:
  class SSHTask : public RobotTask
  {
    using RobotTask::RobotTask;
    bool execute() override;
  };

  Task* perRobotExecution(Context& context, Robot& robot) const override;
};

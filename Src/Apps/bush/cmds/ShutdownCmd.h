#pragma once

#include "cmdlib/RobotCommand.h"

class ShutdownCmd : public RobotCommand<ShutdownCmd>
{
public:
  ShutdownCmd();

private:
  class ShutdownTask : public RobotTask
  {
    using RobotTask::RobotTask;
    bool execute() override;
  };

  Task* perRobotExecution(Context& context, Robot& robot) const override;
};

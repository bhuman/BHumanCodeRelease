#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class ShutdownCmd : public RobotCommand
{
  class ShutdownTask : public RobotTask
  {
  public:
    ShutdownTask(Context& context, Robot* robot);
    bool execute();
  };

  ShutdownCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

public:
  static ShutdownCmd theShutdownCmd;
};

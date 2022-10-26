#pragma once

#include "cmdlib/RobotCommand.h"

class ProcessRunner;

struct RestartArgs : CommandArgs
{
  enum Type
  {
    bhuman,
    robot
  };

  RestartArgs(Type type = bhuman) :
    type(type)
  {}

  Type type;
};

class RestartCmd : public RobotCommand<RestartCmd, RestartArgs>
{
public:
  RestartCmd();

private:
  class RestartTask : public RobotTask
  {
  public:
    RestartTask(Context& context, Robot* robot, const RestartArgs& args);
    bool execute() override;

    const RestartArgs args;
  };

  Task* perRobotExecution(Context& context, Robot& robot, const RestartArgs& args) const override;
};

#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class ProcessRunner;

class PingCmd : public RobotCommand
{
  class PingTask : public RobotTask
  {
    bool isReachable(ProcessRunner &r);
  public:
    PingTask(Context &context, Robot *robot);
    virtual bool execute();
  };

  PingCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual Task* perRobotExecution(Context &context, Robot &robot);
public:
  static PingCmd thePingCmd;
};

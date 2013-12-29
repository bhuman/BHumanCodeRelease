#pragma once

#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/RobotCommand.h"

class DebugRequestCmd;

class DebugRequestCmd : public RobotCommand
{
  std::string request;

  DebugRequestCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, Robot &robot);
public:
  static DebugRequestCmd theDebugRequestCmd;
};

#pragma once

#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/RobotCommand.h"
#include "Utils/bush/models/Team.h"

#include <QString>

class UpdateWirelessCmd : public RobotCommand
{
  class UpdateWirelessTask : public RobotTask
  {
    const std::string command;
  public:
    UpdateWirelessTask(Context &context, Robot *robot, const std::string &command);
    bool execute();
  };

  UpdateWirelessCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, Robot &robot);
public:
  static UpdateWirelessCmd theUpdateWirelessCmd;
};


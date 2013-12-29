#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class ChangeWirelessCmd : public RobotCommand
{
  class ChangeWirelessTask : public RobotTask
  {
    QString config;
  public:
    ChangeWirelessTask(Context &context,
                       Robot *robot,
                       const QString& config);
    bool status;
    bool execute();
    ~ChangeWirelessTask() {}
  };

  QString config;

  ChangeWirelessCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, Robot &robot);
public:
  static ChangeWirelessCmd theChangeWirelessCmd;
};

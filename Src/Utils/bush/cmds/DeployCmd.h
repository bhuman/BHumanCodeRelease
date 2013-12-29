#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class QString;

class DeployCmd : public RobotCommand
{
  Team* team;
  QString buildConfig;

  DeployCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
  virtual bool preExecution(Context &context, const std::vector<std::string> &params);
  virtual Task* perRobotExecution(Context &context, Robot &robot);
  virtual bool postExecution(Context &context, const std::vector<std::string> &params);

  QString getCommand();
public:
  static DeployCmd theDeployCmd;
};

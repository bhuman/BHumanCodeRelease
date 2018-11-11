#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class QString;

class DeployCmd : public RobotCommand
{
  class DeployTask : public RobotTask
  {
    QString buildConfig;
    Team* team;

  public:
    DeployTask(Context& context, const QString& buildConfig, Team* team, Robot* robot);
    bool execute() override;
  };

  Team* team;
  QString buildConfig;

  DeployCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  std::vector<std::string> complete(const std::string& cmdLine) const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

  static QString getCommand();

public:
  static DeployCmd theDeployCmd;
};

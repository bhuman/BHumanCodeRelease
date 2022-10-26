#pragma once

#include "cmdlib/RobotCommand.h"

class QString;

struct DeployArgs : CommandArgs
{
  DeployArgs(const QString& config) :
    config(config)
  {}

  QString config;
};

class DeployCmd : public RobotCommand<DeployCmd, DeployArgs>
{
public:
  DeployCmd();

private:
  class DeployTask : public RobotTask
  {
  public:
    DeployTask(Context& context, const QString& buildConfig, const Team* team, Robot* robot);

  private:
    bool execute() override;
    static QString getCommand();

    const QString buildConfig;
    const Team* team;
  };

  bool preExecution(Context& context, const DeployArgs& args) const override;
  Task* perRobotExecution(Context& context, Robot& robot, const DeployArgs& args) const override;
};

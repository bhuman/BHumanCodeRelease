#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class RestartCmd;
class ProcessRunner;

enum RestartType
{
  BHUMAND, ROBOT, NO_RESTART
};

class RestartCmd : public RobotCommand
{
  class RestartTask : public RobotTask
  {
    RestartType type;
    void reportStatus(const ProcessRunner& r);

  public:
    RestartTask(Context& context, Robot* robot, RestartType type);
    bool execute() override;
  };

  RestartType type;

  RestartCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  std::vector<std::string> complete(const std::string& cmdLine) const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

public:
  static RestartCmd theRestartCmd;
};

#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class ShowCmd : public RobotCommand
{
  std::vector<std::string> files;

  ShowCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  std::vector<std::string> complete(const std::string& cmdLine) const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

  static ShowCmd theShowCmd;
};

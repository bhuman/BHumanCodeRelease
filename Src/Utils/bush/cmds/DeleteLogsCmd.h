/**
 * @file DeleteLogsCmd.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class DeleteLogsCmd : public RobotCommand
{
  class DeleteLogsTask : public RobotTask
  {
  public:
    DeleteLogsTask(Context& context, Robot* robot);
    bool execute() override;
    QString getCommand();
  };

public:
  DeleteLogsCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;

  QString getCommand();

  static DeleteLogsCmd theDeleteLogsCmd;
};

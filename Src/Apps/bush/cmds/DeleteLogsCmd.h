/**
 * @file DeleteLogsCmd.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "cmdlib/RobotCommand.h"

class DeleteLogsCmd : public RobotCommand<DeleteLogsCmd>
{
public:
  DeleteLogsCmd();

private:
  class DeleteLogsTask : public RobotTask
  {
    using RobotTask::RobotTask;
    bool execute() override;
    static QString getCommand();
  };

  Task* perRobotExecution(Context& context, Robot& robot) const override;
};

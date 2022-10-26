/*
 * @file DownloadLogsCmd.h
 * @author Arne Böckmann
 */

#pragma once

#include "cmdlib/RobotCommand.h"

/**
 * Downloads the log files from the robot.
 */
class DownloadLogsCmd : public RobotCommand<DownloadLogsCmd>
{
public:
  DownloadLogsCmd();

private:
  class DownloadLogsTask : public RobotTask
  {
    using RobotTask::RobotTask;
    bool execute() override;
    static QString getCommand();
  };

  Task* perRobotExecution(Context& context, Robot& robot) const override;
};

/*
 * @file DownloadCalibrationCmd.h
 * @author Arne Hasselbring (based on Arne Böckmann's DownloadLogsCmd.h)
 */

#pragma once

#include "cmdlib/RobotCommand.h"

/**
 * Downloads calibration files from the robot.
 */
class DownloadCalibrationCmd : public RobotCommand<DownloadCalibrationCmd>
{
public:
  DownloadCalibrationCmd();

private:
  class DownloadCalibrationTask : public RobotTask
  {
    using RobotTask::RobotTask;
    bool execute() override;
    static QString getCommand();
  };

  Task* perRobotExecution(Context& context, Robot& robot) const override;
};

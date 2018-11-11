/*
 * DownloadLogsCmd.h
 *
 *  Created on: Mar 21, 2012
 *      Author: Arne Böckmann (arneboe@tzi.de)
 */

#ifndef DOWNLOADLOGSCMD_H_
#define DOWNLOADLOGSCMD_H_

#include "Utils/bush/cmdlib/RobotCommand.h"
/**
 * Downloads the log files from the robot.
 */
class DownloadLogsCmd : public RobotCommand
{
  class DownloadLogsTask : public RobotTask
  {
  public:
    DownloadLogsTask(Context& context, Robot* robot);
    bool execute() override;
    QString getCommand();
  };

public:
  DownloadLogsCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool preExecution(Context& context, const std::vector<std::string>& params) override;
  Task* perRobotExecution(Context& context, Robot& robot) override;
  bool postExecution(Context& context, const std::vector<std::string>& params) override;

  QString getCommand();

public:
  static DownloadLogsCmd theDownloadLogsCmd;
};

#endif /* DOWNLOADLOGSCMD_H_ */

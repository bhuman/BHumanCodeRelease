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
    bool execute();
    QString getCommand();
  };

public:
  DownloadLogsCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, Robot& robot);
  virtual bool postExecution(Context& context, const std::vector<std::string>& params);

  QString getCommand();
public:
  static DownloadLogsCmd theDownloadLogsCmd;
};

#endif /* DOWNLOADLOGSCMD_H_ */

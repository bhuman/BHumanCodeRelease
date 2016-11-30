/**
 * @file DeleteLogs.h
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Utils/bush/cmdlib/RobotCommand.h"

class DeleteLogsCmd : public RobotCommand
{
  class DeleteLogsTask : public RobotTask
  {
  public:
    DeleteLogsTask(Context& context, Robot* robot);
    bool execute();
    QString getCommand();
  };

public:
  DeleteLogsCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool preExecution(Context& context, const std::vector<std::string>& params);
  virtual Task* perRobotExecution(Context& context, Robot& robot);

  QString getCommand();

  static DeleteLogsCmd theDeleteLogsCmd;
};

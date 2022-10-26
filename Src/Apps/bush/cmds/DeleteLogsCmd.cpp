/**
 * @file DeleteLogsCmd.cpp
 * @author Andreas Stolpmann
 */

#include "DeleteLogsCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "models/Robot.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <QString>
#include <QStringList>

DeleteLogsCmd::DeleteLogsCmd() :
  RobotCommand("deleteLogs")
{}

Task* DeleteLogsCmd::perRobotExecution(Context& context, Robot& robot) const
{
  return new DeleteLogsCmd::DeleteLogsTask(context, &robot);
}

bool DeleteLogsCmd::DeleteLogsTask::execute()
{
  QStringList args;
  args.push_back("--just-delete");
  args.push_back(QString::fromStdString(robot->getBestIP(context())));

  ProcessRunner r(context(), getCommand(), args);
  r.run();

  if(r.error())
  {
    context().errorLine("Deleting failed!");
    return false;
  }

  context().printLine("Success! (" + QString::fromStdString(robot->name) + ")");
  return true;
}

QString DeleteLogsCmd::DeleteLogsTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs");
#endif
}

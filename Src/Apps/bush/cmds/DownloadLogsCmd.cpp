/*
 * @file DownloadLogsCmd.cpp
 * @author Arne Böckmann
 */

#include "DownloadLogsCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "models/Robot.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <QString>
#include <QStringList>

DownloadLogsCmd::DownloadLogsCmd() :
  RobotCommand("downloadLogs")
{}

Task* DownloadLogsCmd::perRobotExecution(Context& context, Robot& robot) const
{
  return new DownloadLogsCmd::DownloadLogsTask(context, &robot);
}

bool DownloadLogsCmd::DownloadLogsTask::execute()
{
  QStringList args;
  args.push_back(QString::fromStdString(robot->getBestIP(context())));

  ProcessRunner r(context(), getCommand(), args);
  r.run();

  if(r.error())
  {
    context().errorLine("Download failed!");
    return false;
  }

  context().printLine("Success! (" + QString::fromStdString(robot->name) + ")");
  return true;
}

QString DownloadLogsCmd::DownloadLogsTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadLogs");
#endif
}

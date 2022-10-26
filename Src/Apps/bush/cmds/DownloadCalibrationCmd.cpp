/*
 * @file DownloadCalibrationCmd.cpp
 * @author Arne Hasselbring (based on Arne Böckmann's DownloadLogsCmd.cpp)
 */

#include "DownloadCalibrationCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "models/Robot.h"
#include "tools/Platform.h"
#include "Platform/File.h"
#include <QString>
#include <QStringList>

DownloadCalibrationCmd::DownloadCalibrationCmd() :
  RobotCommand("downloadCalibration")
{}

Task* DownloadCalibrationCmd::perRobotExecution(Context& context, Robot& robot) const
{
  return new DownloadCalibrationCmd::DownloadCalibrationTask(context, &robot);
}

bool DownloadCalibrationCmd::DownloadCalibrationTask::execute()
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

QString DownloadCalibrationCmd::DownloadCalibrationTask::getCommand()
{
#ifdef WINDOWS
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadCalibration.cmd");
#else
  return QString::fromStdString(std::string(File::getBHDir()) + "/Make/" + makeDirectory() + "/downloadCalibration");
#endif
}

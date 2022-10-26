#include "ShutdownCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/ShellTools.h"
#include <QString>

ShutdownCmd::ShutdownCmd() :
  RobotCommand("shutdown")
{}

Task* ShutdownCmd::perRobotExecution(Context& context, Robot& robot) const
{
  return new ShutdownTask(context, &robot);
}

bool ShutdownCmd::ShutdownTask::execute()
{
  const std::string ip = robot->getBestIP(context());

  context().printLine(QString::fromStdString(robot->name) + ": Shutting down...");
  const std::string command = remoteCommand("sudo systemctl poweroff", ip);
  ProcessRunner r(context(), QString::fromStdString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(QString::fromStdString(robot->name) + ": Shutdown failed.");
    return false;
  }
  context().printLine(QString::fromStdString(robot->name) + ": Shutdown finished.");
  return true;
}

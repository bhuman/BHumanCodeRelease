#include "RestartCmd.h"
#include "cmdlib/Context.h"
#include "cmdlib/ProcessRunner.h"
#include "tools/ShellTools.h"
#include <QString>

RestartCmd::RestartCmd() :
  RobotCommand("restart")
{}

Task* RestartCmd::perRobotExecution(Context& context, Robot& robot, const RestartArgs& args) const
{
  return new RestartTask(context, &robot, args);
}

RestartCmd::RestartTask::RestartTask(Context& context, Robot* robot, const RestartArgs& args) :
  RobotTask(context, robot),
  args(args)
{}

bool RestartCmd::RestartTask::execute()
{
  const std::string ip = robot->getBestIP(context());

  context().printLine("restart: using ip " + QString::fromStdString(ip) + " for " + QString::fromStdString(robot->name) + ".");

  std::string command;
  if(args.type == RestartArgs::bhuman)
    command = "systemctl --user restart bhuman.service";
  else if(args.type == RestartArgs::robot)
    command = "sudo systemctl reboot";

  ProcessRunner r(context(), remoteCommand(command, ip));
  r.run();
  if(r.error())
  {
    context().errorLine("Robot \"" + QString::fromStdString(robot->name) + "\" is unreachable");
    return false;
  }
  context().printLine("Robot \"" + QString::fromStdString(robot->name) + "\" restarted");
  return true;
}

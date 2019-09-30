#include "Utils/bush/cmds/ShutdownCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include <cstdlib>

ShutdownCmd ShutdownCmd::theShutdownCmd;

ShutdownCmd::ShutdownCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ShutdownCmd::getName() const
{
  return "shutdown";
}

std::string ShutdownCmd::getDescription() const
{
  return "Shuts down the robot.";
}

bool ShutdownCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if(!params.empty() && params[0] != "-s")
  {
    context.errorLine("Unrecognized parameters.");
    return false;
  }
  return true;
}

Task* ShutdownCmd::perRobotExecution(Context& context, Robot& robot)
{
  return new ShutdownTask(context, &robot);
}

ShutdownCmd::ShutdownTask::ShutdownTask(Context& context, Robot* robot)
  : RobotTask(context, robot)
{}

bool ShutdownCmd::ShutdownTask::execute()
{
  std::string ip = robot->getBestIP(context());

  context().printLine(robot->name + ": Shutting down...");
  std::string command = remoteCommand("halt", ip);
  ProcessRunner r(context(), fromString(command));
  r.run();
  if(r.error())
  {
    context().errorLine(robot->name + ": Shutdown failed.");
    return false;
  }
  context().printLine(robot->name + ": Shutdown finished.");
  return true;
}

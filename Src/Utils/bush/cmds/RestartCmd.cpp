#include <cstdlib>
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/RestartCmd.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"
#include <iostream>
RestartCmd RestartCmd::theRestartCmd;

RestartCmd::RestartCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string RestartCmd::getName() const
{
  return "restart";
}

std::string RestartCmd::getDescription() const
{
  return std::string("[ bhuman | robot ]\n")
         + "Restarts bhumand or the robot.";
}

bool RestartCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  type = NO_RESTART;
  if(params.size() > 1)
  {
    context.errorLine("Too many parameters.");
    return false;
  }
  else if(params.size() == 0 || params[0] == "bhuman")
    type = BHUMAND;
  else if(params[0] == "robot")
    type = ROBOT;

  return true;
}

Task* RestartCmd::perRobotExecution(Context& context, Robot& robot)
{
  return new RestartTask(context, &robot, type);
}

std::vector<std::string> RestartCmd::complete(const std::string& cmdLine) const
{
  const size_t ARGS_SIZE = 2;
  std::string arguments[ARGS_SIZE] = { "bhuman", "robot" };

  std::vector<std::string> result;
  std::vector<std::string> commandWithArgs = split(cmdLine);
  if(commandWithArgs.size() == 2)
  {
    for(size_t i = 0; i < ARGS_SIZE; i++)
    {
      if(arguments[i].find(commandWithArgs[1]) == 0)
        result.push_back(arguments[i].substr(commandWithArgs[1].size(), arguments[i].size()));
    }
  }
  return result;
}

RestartCmd::RestartTask::RestartTask(Context& context, Robot* robot, RestartType type)
  : RobotTask(context, robot),
    type(type)
{}

void RestartCmd::RestartTask::reportStatus(const ProcessRunner& r)
{
  if(r.error())
    context().errorLine("Robot \"" + robot->name + "\" is unreachable");
  else
    context().printLine("Robot \"" + robot->name + "\" restarted");
}

bool RestartCmd::RestartTask::execute()
{
  std::string ip = robot->getBestIP(context());

  context().printLine("restart: using ip " + ip + " for " + robot->name + ".");

  if(type == BHUMAND)
  {
    std::string command = remoteCommand("bhumand restart", ip);
    ProcessRunner r(context(), command);
    r.run();
    reportStatus(r);
  }
  else if(type == ROBOT)
  {
    std::string command = remoteCommand("reboot", ip);
    ProcessRunner r(context(), command);
    r.run();
    reportStatus(r);
  }
  else
  {
    context().errorLine("Unkown restart command.");
    return false;
  }

  return true;
}

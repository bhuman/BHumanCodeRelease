#include <cstdlib>
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/RestartCmd.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/Sleeper.h"
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
  return std::string("[ bhuman | naoqi | full | robot ]\n")
         + "Restarts bhumand, naoqid, both or the robot.";
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
  else if(params[0] == "naoqi")
    type = NAOQID;
  else if(params[0] == "robot")
    type = ROBOT;
  else if(params[0] == "full")
    type = BHUMAND_AND_NAOQID;

  return true;
}

Task* RestartCmd::perRobotExecution(Context& context, Robot& robot)
{
  return new RestartTask(context, &robot, type);
}

std::vector<std::string> RestartCmd::complete(const std::string& cmdLine) const
{
  const size_t ARGS_SIZE = 4;
  std::string arguments[ARGS_SIZE] = { "bhuman", "naoqi", "full", "robot" };

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

  if(type < SINGLE_COMMANDS)
  {
    if(type == BHUMAND)
    {
      std::string command = remoteCommand("bhumand restart", ip);
      ProcessRunner r(context(), command);
      r.run();
      reportStatus(r);
    }
    else if(type == NAOQID)
    {
      std::string command = remoteCommand("sudo /etc/init.d/naoqi restart", ip);
      ProcessRunner r(context(), command);
      r.run();
      if(r.error())
      {
        context().errorLine(robot->name + ": Could not restart Naoqi.");
        return false;
      }
      else context().printLine(robot->name + ": restarted Naoqi.");
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
  }
  else if(type == BHUMAND_AND_NAOQID)
  {
    std::string command = remoteCommand("bhumand stop", ip);
    ProcessRunner r(context(), command);
    r.run();
    if(r.error())
    {
      context().errorLine(robot->name + ": Failed to stop bhumand.");
      return false;
    }
    else
    {
      context().printLine(robot->name + ": bhumand stopped");
    }
    context().printLine(robot->name + ": waiting 2 seconds");
    Sleeper::msleep(2000);

    command = remoteCommand("sudo /etc/init.d/naoqi restart", ip);
    r = ProcessRunner(context(), command);
    r.run();
    if(r.error())
    {
      context().errorLine(robot->name + ": Failed to restart naoqid.");
      return false;
    }
    else context().printLine(robot->name + ": naoqid restarted");
    context().printLine(robot->name + ": waiting 2 seconds");
    Sleeper::msleep(2000);

    command = remoteCommand("bhumand start", ip);
    r = ProcessRunner(context(), command);
    r.run();
    if(r.error())
    {
      context().errorLine(robot->name + ": Failed to start bhumand.");
      return false;
    }
    else context().printLine(robot->name + ": bhumand started");
    context().printLine(robot->name + ": waiting 2 seconds");
    Sleeper::msleep(2000);
    context().printLine(robot->name + ": Done");
  }
  else
  {
    context().errorLine("Unkown restart command.");
    return false;
  }

  return true;
}

#include "Utils/bush/cmds/DebugRequestCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/StringTools.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/Platform.h"
#include "Utils/bush/Session.h"
#include "Platform/File.h"
#include <cstdlib>

DebugRequestCmd DebugRequestCmd::theDebugRequestCmd;

DebugRequestCmd::DebugRequestCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string DebugRequestCmd::getName() const
{
  return "dr";
}

std::string DebugRequestCmd::getDescription() const
{
  return "Sends debug requests.";
}

bool DebugRequestCmd::preExecution(Context &context, const std::vector<std::string> &params)
{
  if(params.size() >= 1)
    request = "dr " + params[0];
  if(params.size() == 2)
    request += " " + params[1];
  else if(params.size() > 2)
  {
    context.errorLine("Too many parameters.");
    return false;
  }

  return true;
}

Task* DebugRequestCmd::perRobotExecution(Context &context, Robot &robot)
{
  std::vector<std::string> answer = Session::getInstance().sendDebugRequest(&robot, request);
  if(answer.empty())
  {
    context.errorLine(robot.name + ": Could not send debug request.");
  }
  else
  {
    context.printLine(robot.name + ": ");
    for(size_t i = 0; i < answer.size(); i++)
      context.printLine("\t" + answer[i]);
  }

  return NULL;
}

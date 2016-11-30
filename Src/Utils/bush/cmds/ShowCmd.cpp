#include <string>
#include <algorithm>
#include "Platform/File.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/ProcessRunner.h"
#include "Utils/bush/cmds/ShowCmd.h"
#include "Utils/bush/tools/Filesystem.h"
#include "Utils/bush/tools/ShellTools.h"
#include "Utils/bush/tools/StringTools.h"

ShowCmd::ShowCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string ShowCmd::getName() const
{
  return "show";
}

std::string ShowCmd::getDescription() const
{
  return "Prints the content of config files of selected robots.";
}

std::vector<std::string> ShowCmd::complete(const std::string& cmdLine) const
{
  std::vector<std::string> commandWithArgs = split(cmdLine);

  std::string prefix = commandWithArgs.size() == 1 ? "" : commandWithArgs[1];
  std::string dir = std::string(File::getBHDir()) + "/Config/" + prefix;

  std::vector<std::string> files = Filesystem::getEntries(dir,
                                   true,   // files
                                   false,  // dirs
                                   ".cfg");
  std::vector<std::string> dirs = Filesystem::getEntries(dir,
                                  false,   // files
                                  true);   // dirs

  for(size_t i = 0; i < dirs.size(); ++i)
    files.push_back(dirs[i] + "/");
  return files;
}

bool ShowCmd::preExecution(Context& context, const std::vector<std::string>& params)
{
  if(params.empty())
  {
    context.errorLine("No config file specified.");
    return false;
  }
  files = params;

  return true;
}

Task* ShowCmd::perRobotExecution(Context& context, Robot& robot)
{
  context.printLine("--------------");
  context.printLine(robot.name + ":");
  context.printLine("--------------");

  for(size_t i = 0; i < files.size(); ++i)
  {
    context.printLine("-- " + files[i] + ":");
    std::string cmd = remoteCommandForQProcess(
                        "cat /home/nao/Config/" + files[i], robot.getBestIP(context));

    ProcessRunner r(context, cmd);
    r.run();
    //TODO: maybe we want a separator here
    if(r.error())
      context.errorLine("show \"" + files[i] + "\" on \"" + robot.name + "\" failed!");
    context.printLine("");
  }

  return nullptr;
}

ShowCmd ShowCmd::theShowCmd;

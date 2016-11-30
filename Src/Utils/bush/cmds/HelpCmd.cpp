#include "Utils/bush/cmds/HelpCmd.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/tools/StringTools.h"

HelpCmd HelpCmd::theHelpCmd;

HelpCmd::HelpCmd()
{
  Commands::getInstance().addCommand(this);
}

std::string HelpCmd::getName() const
{
  return "help";
}

std::string HelpCmd::getDescription() const
{
  return "Print this help message.";
}

bool HelpCmd::execute(Context& context, const std::vector<std::string>& params)
{
  if(params.empty())
  {
    std::vector<Command*> allCmds = Commands::getInstance().getAllCommands();
    for(std::vector<Command*>::iterator i = allCmds.begin(); i != allCmds.end(); ++i)
      context.printLine((*i)->getName() + ":\n\t" +
                        toString(fromString((*i)->getDescription()).replace("\n", "\n\t")));
  }
  else
    for(size_t i = 0; i < params.size(); ++i)
    {
      Command* cmd = Commands::getInstance()[params[i]];
      if(cmd)
        context.printLine(cmd->getName() + ":\t" + cmd->getDescription());
      else
        context.errorLine(params[i] + ":\t<not found>");
    }
  return true;
}

std::vector<std::string> HelpCmd::complete(const std::string& cmdLine) const
{
  // TODO: complete last begun parameter
  std::vector<std::string> commandWithArgs = split(cmdLine);
  std::vector<std::string> result;
  size_t lastIdx = commandWithArgs.size() - 1;

  // indicates that we want to find a new parameter
  if(*(--cmdLine.end()) == ' ')
  {
    ++lastIdx;
    commandWithArgs.push_back("");
  }

  std::vector<std::string> cmds = Commands::getInstance().getAllCommandNames();
  for(size_t i = 0; i < cmds.size(); ++i)
  {
    if(startsWidth(cmds[i], commandWithArgs[lastIdx]))
    {
      std::vector<std::string> v = commandWithArgs;
      v[lastIdx] = cmds[i];
      std::string s = join(v);
      result.push_back(s);
    }
  }

  return result;
}

#include "Utils/bush/cmdlib/Commands.h"
#include "Utils/bush/cmdlib/Command.h"
#include "Utils/bush/cmdlib/Context.h"
#include "Utils/bush/tools/StringTools.h"

Commands& Commands::getInstance()
{
  static Commands theInstance;
  return theInstance;
}

bool Commands::addCommand(Command* cmd)
{
  std::map<std::string, Command*>::iterator iter = commands.find(cmd->getName());
  if(iter == commands.end())
    commands.insert(commands.begin(), std::pair<std::string, Command*>(cmd->getName(), cmd));
  else
    return false;
  return true;
}

bool Commands::removeCommand(const Command* cmd)
{
  return commands.erase(cmd->getName()) > 0;
}

bool Commands::removeCommand(const std::string& name)
{
  return commands.erase(name) > 0;
}

bool Commands::execute(Context* context, const std::string& cmdLine)
{
  std::vector<std::string> splitted = split(cmdLine);
  if(splitted.size() < 1)
  {
    context->errorLine("Syntax Error ;-)");
    return false;
  }
  std::map<std::string, Command*>::iterator iter = commands.find(splitted[0]);
  if(iter == commands.end())
  {
    context->errorLine(splitted[0] + " not found");
    return false;
  }
  std::vector<std::string> parameters;
  if(splitted.size() > 1)
  {
    parameters.reserve(splitted.size() - 1);
    for(size_t i = 1; i < splitted.size(); ++i)
      parameters.push_back(splitted[i]);
  }
  return iter->second->execute(*context, parameters);
}

Command* Commands::operator[](const std::string& cmd)
{
  std::map<std::string, Command*>::iterator iter = commands.find(cmd);
  if(iter == commands.end())
    return nullptr;
  return iter->second;
}

std::vector<Command*> Commands::getAllCommands()
{
  std::vector<Command*> all;
  all.reserve(commands.size());
  for(std::map<std::string, Command*>::const_iterator i = commands.begin();
      i != commands.end();
      ++i)
    all.push_back(i->second);
  return all;
}

std::vector<std::string> Commands::getAllCommandNames() const
{
  std::vector<std::string> all;
  all.reserve(commands.size());
  for(std::map<std::string, Command*>::const_iterator i = commands.begin();
      i != commands.end();
      ++i)
    all.push_back(i->first);
  return all;
}

std::vector<std::string> Commands::complete(const std::string& cmdLine)
{
  std::vector<std::string> commandWithArgs = split(cmdLine);
  std::vector<std::string> result;
  if(commandWithArgs.size() > 1 || (!cmdLine.empty() && *(--cmdLine.end()) == ' '))
  {
    // parameter completion (this is the job of the individual Command instaces)
    std::map<std::string, Command*>::const_iterator i = commands.find(commandWithArgs[0]);
    if(i != commands.end())
    {
      std::vector<std::string> completionResult = i->second->complete(cmdLine);
      result.insert(result.begin(), completionResult.begin(), completionResult.end());
    }

    /* For backward compatibility: Check if the completion strings start with the
     * given command and prepend it if not. Assumes that the prefix is missing
     * in every string if it is missing in the first one.
     */
    if(!result.empty() && !startsWidth(result[0], commandWithArgs[0]))
    {
      for(size_t i = 0; i < result.size(); ++i)
        result[i] = commandWithArgs[0] + " " + result[i];
    }
  }
  else
  {
    // command completion
    for(std::map<std::string, Command*>::const_iterator i = commands.begin();
        i != commands.end(); i++)
    {
      if(startsWidth(i->first, cmdLine))
        result.push_back(i->first);
    }
  }
  return result;
}

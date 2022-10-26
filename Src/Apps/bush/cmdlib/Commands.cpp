#include "Commands.h"
#include "cmdlib/Command.h"

Commands& Commands::getInstance()
{
  static Commands theInstance;
  return theInstance;
}

bool Commands::add(CommandBase* cmd)
{
  if(auto iter = commands.find(cmd->name); iter != commands.end())
    return false;
  commands[cmd->name] = cmd;
  return true;
}

CommandBase* Commands::get(const std::string& name) const
{
  if(auto iter = commands.find(name); iter != commands.end())
    return iter->second;
  return nullptr;
}

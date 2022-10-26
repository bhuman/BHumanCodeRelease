#pragma once

#include <map>
#include <string>

class CommandBase;

class Commands
{
public:
  static Commands& getInstance();
  bool add(CommandBase* cmd);
  CommandBase* get(const std::string& name) const;

private:
  std::map<std::string, CommandBase*> commands;
};

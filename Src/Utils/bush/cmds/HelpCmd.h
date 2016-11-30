#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"

class HelpCmd;

class HelpCmd : public CommandAdapter
{
  HelpCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
  virtual std::vector<std::string> complete(const std::string& cmdLine) const;
public:
  static HelpCmd theHelpCmd;
};

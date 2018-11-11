#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"

class HelpCmd;

class HelpCmd : public CommandAdapter
{
  HelpCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool execute(Context& context, const std::vector<std::string>& params) override;
  std::vector<std::string> complete(const std::string& cmdLine) const override;

public:
  static HelpCmd theHelpCmd;
};

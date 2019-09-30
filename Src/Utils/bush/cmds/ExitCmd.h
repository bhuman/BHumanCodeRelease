#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"

class ExitCmd;

class ExitCmd : public CommandAdapter
{
  ExitCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool execute(Context& context, const std::vector<std::string>& params) override;

public:
  static ExitCmd theExitCmd;
};

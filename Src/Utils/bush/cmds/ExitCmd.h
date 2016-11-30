#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"

class ExitCmd;

class ExitCmd : public CommandAdapter
{
  ExitCmd();
  virtual std::string getName() const;
  virtual std::string getDescription() const;
  virtual bool execute(Context& context, const std::vector<std::string>& params);
public:
  static ExitCmd theExitCmd;
};

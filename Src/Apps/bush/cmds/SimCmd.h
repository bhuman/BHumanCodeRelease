#pragma once

#include "cmdlib/Command.h"
#include <string>

class SimCmd : public Command<SimCmd>
{
public:
  SimCmd();

private:
  bool execute(Context& context) const override;
  static std::string getSimulatorExecutable(const std::string& buildConfig);
};

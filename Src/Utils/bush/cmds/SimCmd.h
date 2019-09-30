#pragma once

#include "Utils/bush/cmdlib/CommandAdapter.h"

class SimCmd : public CommandAdapter
{
  SimCmd();
  std::string getName() const override;
  std::string getDescription() const override;
  bool execute(Context& context, const std::vector<std::string>& params) override;
  std::string getSimulatorExecutable(const std::string& buildConfig);

public:
  static SimCmd theSimCmd;
};

#include "Utils/bush/cmdlib/CommandAdapter.h"

std::string CommandAdapter::getDescription() const
{
  return "";
}

std::vector<std::string> CommandAdapter::complete(const std::string&) const
{
  return std::vector<std::string>();
}

#pragma once

#include <string>

class IConsole
{
public:
  virtual ~IConsole() = default;
  virtual void print(const std::string& msg) = 0;
  virtual void printLine(const std::string& msg) = 0;
  virtual void error(const std::string& msg) = 0;
  virtual void errorLine(const std::string& msg) = 0;
};

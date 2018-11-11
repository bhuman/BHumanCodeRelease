#pragma once

#include <string>
#include <map>

/**
 * A wrapper for the B-Human framework. Create one instance and you do not have
 * to worry about any initializations. This is implemented as a singleton.
 */
class Framework
{
  static std::map<std::string, Framework*> theInstances;

  Framework(const std::string& processName);

public:
  static Framework* getInstance(const std::string& processName);
  static void destroy(const std::string& processName);
};

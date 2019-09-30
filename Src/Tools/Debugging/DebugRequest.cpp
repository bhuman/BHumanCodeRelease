/**
 * @file DebugRequest.cpp
 *
 * Implementation of a class that manages debug requests.
 * Debug requests are switches that (de)activate blocks of code at runtime.
 *
 * @author Matthias Jüngel
 * @author Thomas Röfer
 */

#include <cstdio>

#include "DebugRequest.h"
#include "Platform/BHAssert.h"

DebugRequestTable::DebugRequestTable()
{
  enabled.reserve(10000);
  fastIndex.reserve(10000);
  slowIndex.reserve(10000);
  polled.reserve(10000);
}

void DebugRequestTable::addRequest(const DebugRequest& debugRequest)
{
  if(debugRequest.name == "poll")
  {
    pollCounter = 3;
    polled.clear();
  }
  else if(debugRequest.name == "disableAll")
    clear();
  else
  {
    std::unordered_map<std::string, size_t>::const_iterator i = slowIndex.find(debugRequest.name);
    if(i != slowIndex.end())
      enabled[i->second] = debugRequest.enable ? 1 : 0;
    else
    {
      slowIndex[debugRequest.name] = enabled.size();
      enabled.push_back(debugRequest.enable ? 1 : 0);
    }
  }
}

bool DebugRequestTable::isActiveSlow(const char* name)
{
  std::unordered_map<std::string, size_t>::const_iterator j = slowIndex.find(name);
  size_t k;
  if(j != slowIndex.end())
    k = j->second;
  else
  {
    k = enabled.size();
    slowIndex[name] = k;
    enabled.push_back(0);
  }
  fastIndex[name] = k;
  return enabled[k] != 0;
}

void DebugRequestTable::disable(const char* name)
{
  ASSERT(fastIndex.find(name) != fastIndex.end());
  enabled[fastIndex[name]] = 0;
}

bool DebugRequestTable::notYetPolled(const char* name)
{
  if(polled.find(name) == polled.end())
  {
    polled.insert(name);
    return true;
  }
  else
    return false;
}

void DebugRequestTable::clear()
{
  fastIndex.clear();
  slowIndex.clear();
  enabled.clear();
}

void DebugRequestTable::print(const char* message)
{
  fprintf(stderr, "%s\n", message);
}

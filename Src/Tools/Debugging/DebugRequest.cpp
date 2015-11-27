/**
 * @file DebugRequest.cpp
 * Implementation of class DebugRequest
 */

#include <cstring>
#include <cstdio>

#include "DebugRequest.h"
#include "Platform/BHAssert.h"

DebugRequest::DebugRequest(const std::string& description, bool enable) :
  description(description), enable(enable)
{}

void DebugRequestTable::addRequest(const DebugRequest& debugRequest, bool force)
{
  lastName = 0;
  nameToIndex.clear();
  if(debugRequest.description == "poll")
  {
    poll = true;
    pollCounter = 0;
    alreadyPolledDebugRequestCounter = 0;
  }
  else if(debugRequest.description == "disableAll")
    removeAllRequests();
  else
  {
    for(int i = 0; i < currentNumberOfDebugRequests; i++)
    {
      if(debugRequest.description == debugRequests[i].description)
      {
        if(!debugRequest.enable && !force)
          debugRequests[i] = debugRequests[--currentNumberOfDebugRequests];
        else
          debugRequests[i] = debugRequest;
        return;
      }
    }
    if(debugRequest.enable || force)
    {
      ASSERT(currentNumberOfDebugRequests < maxNumberOfDebugRequests);
      debugRequests[currentNumberOfDebugRequests++] = debugRequest;
    }
  }
}

void DebugRequestTable::disable(const char* name)
{
  lastName = 0;
  nameToIndex.clear();
  for(int i = 0; i < currentNumberOfDebugRequests; i++)
    if(debugRequests[i].description == name)
    {
      debugRequests[i] = debugRequests[--currentNumberOfDebugRequests];
      return;
    }
}

bool DebugRequestTable::notYetPolled(const char* name)
{
  for(int i = 0; i < alreadyPolledDebugRequestCounter; ++i)
    if(strcmp(name, alreadyPolledDebugRequests[i]) == 0)
      return false;
  alreadyPolledDebugRequests[alreadyPolledDebugRequestCounter++] = name;
  return true;
}

In& operator>>(In& stream, DebugRequest& debugRequest)
{
  return stream >> debugRequest.enable >> debugRequest.description;
}

Out& operator<<(Out& stream, const DebugRequest& debugRequest)
{
  return stream << debugRequest.enable << debugRequest.description;
}

void DebugRequestTable::print(const char* message)
{
  fprintf(stderr, "%s\n", message);
}

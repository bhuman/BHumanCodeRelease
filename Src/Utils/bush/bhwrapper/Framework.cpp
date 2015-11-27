#include "Framework.h"
#include "Utils/bush/Session.h"
#include "Platform/BHAssert.h"
#include "Tools/Global.h"

std::map<std::string, Framework*> Framework::theInstances;

Framework* Framework::getInstance(const std::string& processName)
{
  if(!theInstances[processName])
    theInstances[processName] = new Framework(processName);
  return theInstances[processName];
}

void Framework::destroy(const std::string& processName)
{
  delete theInstances[processName];
  theInstances.erase(processName);
  Session::getInstance().log(TRACE, "Framework: Deleted " + processName + ".");
}

Framework::Framework(const std::string& processName)
{
  if(theInstances[processName])
  {
    Session::getInstance().log(CRITICAL, "Framework: Thread " + processName
                               + " already defined, could not start TeamCommAgent.");
  }
  ASSERT(!theInstances[processName]);
  Global::theStreamHandler = &streamHandler; // overwrite handler set by settings

  Session::getInstance().log(TRACE, "Framework: Initialized " + processName
                             + ".");
}

Framework::~Framework()
{
  Global::theStreamHandler = 0;
}

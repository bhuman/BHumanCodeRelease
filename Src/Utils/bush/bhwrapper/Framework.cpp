#include "Framework.h"
#include "Utils/bush/Session.h"
#include "Tools/Global.h"
#include "Platform/BHAssert.h"
#include "Representations/Infrastructure/RoboCupGameControlData.h"

std::map<std::string, Framework*> Framework::theInstances;

Framework* Framework::getInstance(const std::string& processName,
                                  int playerNumber, unsigned short port)
{
  if(!theInstances[processName])
    theInstances[processName] = new Framework(processName, playerNumber, port);
  return theInstances[processName];
}

void Framework::destroy(const std::string& processName)
{
  delete theInstances[processName];
  theInstances.erase(processName);
  Session::getInstance().log(TRACE, "Framework: Deleted " + processName + ".");
}

Framework::Framework(const std::string& processName, int playerNumber,
                     unsigned short port) : playerNumber(playerNumber)
{
  if(theInstances[processName])
  {
    Session::getInstance().log(CRITICAL, "Framework: Thread " + processName
                               + " already defined, could not start TeamCommAgent.");
  }
  ASSERT(!theInstances[processName]);
  if(!Settings::loaded)
  {
    Session::getInstance().log(CRITICAL, "Framework: Settings not loaded ("
                               + processName + ").");
  }
  ASSERT(Settings::loaded);
  settings.playerNumber = playerNumber;
  settings.teamPort = port;
  settings.robot = "bush";
  Global::theDebugOut = &debugOut.out;
  Global::theTeamOut = &teamOut.out;
  if(!Global::theSettings)
    Global::theSettings = &settings;
  else
  {
    Session::getInstance().log(WARN,
                               "Framework: Global Settings already defined (" + processName + ").");
  }
  if(!Global::theDebugRequestTable)
    Global::theDebugRequestTable = &debugRequestTable;
  else
  {
    Session::getInstance().log(WARN,
                               "Framework: Global DebugRequestTable already defined (" + processName
                               + ").");
  }
  if(!Global::theDebugDataTable)
    Global::theDebugDataTable = &debugDataTable;
  else
  {
    Session::getInstance().log(WARN,
                               "Framework: Global DebugDataTable already defined (" + processName
                               + ").");
  }

  Global::theStreamHandler = &streamHandler; // overwrite handler set by settings
  Global::theDrawingManager = 0; // we don't need any drawings here

  Session::getInstance().log(TRACE, "Framework: Initialized " + processName
                             + ".");
}

Framework::~Framework()
{
  Global::theSettings = 0;
  Global::theDebugRequestTable = 0;
  Global::theDebugDataTable = 0;
  Global::theStreamHandler = 0;
  Global::theDrawingManager = 0;
}

Framework::Framework(const Framework& framework)
{
  Session::getInstance().log(CRITICAL, "Framework: Standard constructor called.");
}

Framework::Framework()
{
  Session::getInstance().log(CRITICAL, "Framework: Copy constructor called.");
}

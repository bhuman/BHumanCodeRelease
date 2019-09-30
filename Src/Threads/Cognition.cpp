/**
 * @file Threads/Cognition.cpp
 *
 * This file implements the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 */

#include "Modules/Infrastructure/InterThreadProviders/PerceptionProviders.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

REGISTER_EXECUTION_UNIT(Cognition)

thread_local bool Cognition::isUpper = false;

Cognition::Cognition()
: theSPLMessageHandler(inTeamMessages, outTeamMessage)
{
#ifdef TARGET_SIM
  theSPLMessageHandler.startLocal(Global::getSettings().teamPort, static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  std::string bcastAddr = UdpComm::getWifiBroadcastAddress();
  theSPLMessageHandler.start(Global::getSettings().teamPort, bcastAddr.c_str());
#endif
  Blackboard::getInstance().alloc<UpperFrameInfo>("UpperFrameInfo").time = 100000;
  Blackboard::getInstance().alloc<LowerFrameInfo>("LowerFrameInfo").time = 100000;
}

Cognition::~Cognition()
{
  Blackboard::getInstance().free("UpperFrameInfo");
  Blackboard::getInstance().free("LowerFrameInfo");
}

bool Cognition::beforeFrame()
{
  // read from team comm udp socket
  static_cast<void>(theSPLMessageHandler.receive());

  const FrameInfo* lowerFrameInfo = Blackboard::getInstance().exists("LowerFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["LowerFrameInfo"]))
                                    : nullptr;
  const FrameInfo* upperFrameInfo = Blackboard::getInstance().exists("UpperFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["UpperFrameInfo"]))
                                    : nullptr;
  unsigned lowerFrameTime = lowerFrameInfo ? lowerFrameInfo->time : 0;
  unsigned upperFrameTime = upperFrameInfo ? upperFrameInfo->time : 0;

  upperIsNew |= upperFrameTime != lastUpperFrameTime && (upperFrameTime >= lastAcceptedTime || SystemCall::getMode() == SystemCall::logFileReplay);
  lowerIsNew |= lowerFrameTime != lastLowerFrameTime && (lowerFrameTime >= lastAcceptedTime || SystemCall::getMode() == SystemCall::logFileReplay);
  lastUpperFrameTime = upperFrameTime;
  lastLowerFrameTime = lowerFrameTime;

  // Begin a new frame if either there is data for a new one left over from
  // the previous frame or we have two from which we can choose.
  if(acceptNext || (upperIsNew && lowerIsNew))
  {
    // We always switch between upper and lower
    isUpper ^= true;
    if(isUpper)
    {
      lastAcceptedTime = upperFrameTime;
      upperIsNew = false;

      // The other frame can only be used if not older than the current one.
      lowerIsNew &= upperFrameTime <= lowerFrameTime;
    }
    else
    {
      lastAcceptedTime = lowerFrameTime;
      lowerIsNew = false;

      // The other frame can only be used if not older than the current one.
      upperIsNew &= lowerFrameTime <= upperFrameTime;
    }

    // The other frame should also be processed immediately if there is one left.
    acceptNext = (isUpper && lowerIsNew) || (!isUpper && upperIsNew);
    return true;
  }
  else
  {
    // Wait for another frame to arrive before one can be picked.
    // However, a new frame is accepted when replaying logs.
    if(LogDataProvider::exists() && LogDataProvider::isFrameDataComplete()
       && SystemCall::getMode() == SystemCall::logFileReplay)
      return true;
  }

  return false;
}

void Cognition::beforeModules()
{
  BH_TRACE_MSG("before TeamData");
  // push teammate data in our system
  if(Blackboard::getInstance().exists("TeamData") &&
     static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate)
  {
    while(!inTeamMessages.empty())
      static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate(inTeamMessages.takeBack());
  }

  DECLARE_PLOT("module:SPLMessageHandler:standardMessageDataBufferUsageInPercent");
}

void Cognition::afterModules()
{
  if(Blackboard::getInstance().exists("BHumanMessageOutputGenerator")
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).generate
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame)
  {
    static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).generate(&outTeamMessage);

    BH_TRACE_MSG("before theSPLMessageHandler.send()");
    theSPLMessageHandler.send();
  }
}

bool Cognition::afterFrame()
{
  // If there is already a frame waiting to be processed, do not wait for the next one to arrive.
  return !acceptNext;
}

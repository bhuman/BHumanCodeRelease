/**
 * @file Threads/Cognition.cpp
 *
 * This file implements the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 */

#include "Cognition.h"
#include "Modules/Infrastructure/InterThreadProviders/PerceptionProviders.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Representations/Communication/BHumanMessageOutputGenerator.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

REGISTER_EXECUTION_UNIT(Cognition)

thread_local bool Cognition::isUpper = false;

Cognition::Cognition()
{
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
  // Currently replaying a log file?
  const bool replay = (SystemCall::getMode() == SystemCall::logFileReplay || SystemCall::getMode() == SystemCall::remoteRobot)
                      && LogDataProvider::exists();

  // During replay if there is no new frame and no delayed frame, there is nothing to process
  if(replay && !LogDataProvider::isFrameDataComplete(false) && !delayedLogCounter)
  {
    acceptNext = false;
    return false;
  }

  const FrameInfo* lowerFrameInfo = Blackboard::getInstance().exists("LowerFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["LowerFrameInfo"]))
                                    : nullptr;
  const FrameInfo* upperFrameInfo = Blackboard::getInstance().exists("UpperFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["UpperFrameInfo"]))
                                    : nullptr;
  unsigned lowerFrameTime = lowerFrameInfo ? lowerFrameInfo->time : 0;
  unsigned upperFrameTime = upperFrameInfo ? upperFrameInfo->time : 0;

  const bool upperIsLate = static_cast<int>(lowerFrameTime - upperFrameTime) >= 100;
  const bool lowerIsLate = static_cast<int>(upperFrameTime - lowerFrameTime) >= 100;
  upperIsNew |= upperFrameTime != lastUpperFrameTime && upperFrameTime >= lastAcceptedTime;
  lowerIsNew |= lowerFrameTime != lastLowerFrameTime && lowerFrameTime >= lastAcceptedTime;
  lastUpperFrameTime = upperFrameTime;
  lastLowerFrameTime = lowerFrameTime;

  // Begin a new frame if either there is data for a new one left over from
  // the previous frame or we have two from which we can choose.
  if(acceptNext || (upperIsNew && lowerIsNew) || (upperIsNew && lowerIsLate) || (lowerIsNew && upperIsLate))
  {
    // The frame will be processed, so no delay and no data waiting anymore
    delayedLogCounter = 0;

    // During replay, acknowledge that a new frame was received
    if(replay)
      LogDataProvider::isFrameDataComplete();

    // We switch between upper and lower except if one of them is really late
    if(upperIsLate)
      isUpper = false;
    else if(lowerIsLate)
      isUpper = true;
    else
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
  else if(replay)
  {
    // Give Upper and Lower time to process the data. However, since logs can be
    // replayed at different speeds, this code is not bullet-proof. There must
    // be a timeout, because a frame from Upper or Lower can actually be missing
    // and the log would get stuck.
    Thread::sleep(3);
    if(++delayedLogCounter >= 11)
    {
      LogDataProvider::isFrameDataComplete();
      delayedLogCounter = 1;
      lastAcceptedTime = 0;
    }
  }

  return false;
}

void Cognition::beforeModules()
{
  BHExecutionUnit::beforeModules();
  if(SystemCall::getMode() != SystemCall::physicalRobot)
  {
    DEBUG_RESPONSE_NOT("unmute")
      SystemCall::mute(true);
    else
      SystemCall::mute(false);
  }
}

void Cognition::afterModules()
{
  if(Blackboard::getInstance().exists("BHumanMessageOutputGenerator")
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).send
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame
     && static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).sendThisFrame())
  {
    BH_TRACE_MSG("before BHumanMessageOutputGenerator::send()");
    static_cast<const BHumanMessageOutputGenerator&>(Blackboard::getInstance()["BHumanMessageOutputGenerator"]).send();
  }
}

bool Cognition::afterFrame()
{
  // If there is already a frame waiting to be processed, do not wait for the next one to arrive.
  return !acceptNext && !delayedLogCounter;
}

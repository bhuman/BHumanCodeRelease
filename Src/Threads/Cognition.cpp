/**
 * @file Threads/Cognition.cpp
 *
 * This file implements the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 */

#include "Cognition.h"
#include "Debugging/DebugRequest.h"
#include "Framework/Settings.h"
#include "Modules/Infrastructure/InterThreadProviders/PerceptionProviders.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Representations/Communication/BHumanMessageOutputGenerator.h"
#include "Streaming/Global.h"

REGISTER_EXECUTION_UNIT(Cognition)

Cognition::Cognition()
{
  Blackboard::getInstance().alloc<ColorFrameInfo>("ColorFrameInfo").time = minTime;
}

Cognition::~Cognition()
{
  Blackboard::getInstance().free("ColorFrameInfo");
}

bool Cognition::beforeFrame()
{
  // Currently replaying a log file?
  const bool replay = SystemCall::getMode() == SystemCall::logFileReplay
                      && LogDataProvider::exists();

  // During replay if there is no new frame and no delayed frame, there is nothing to process
  if(replay && !LogDataProvider::isFrameDataComplete(false) && !delayedLogCounter)
  {
    return false;
  }

  const unsigned frameTime = static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()
    ["ColorFrameInfo"]))->time;

  // Begin a new frame if either there is data for a new one left over from
  // the previous frame or we have two from which we can choose.
  if(frameTime != lastFrameTime)
  {
    lastFrameTime = frameTime;

    // The frame will be processed, so no delay and no data waiting anymore
    delayedLogCounter = 0;

    // During replay, acknowledge that a new frame was received
    if(replay)
      LogDataProvider::isFrameDataComplete();

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
      return Global::getDebugRequestTable().pollCounter > 0;
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
  return !delayedLogCounter;
}

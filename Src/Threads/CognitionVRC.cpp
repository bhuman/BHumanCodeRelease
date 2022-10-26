/**
 * @file CognitionVRC.cpp
 *
 * This file implements a thread that contains modeling and
 * behavior control for the Visual Referee Challenge. It does
 * not force to alternate beween both perception threads.
 * Instead it prefers the Upper thread (which will send less
 * often), but also accepts everything from the Lower thread.
 * FrameInfo.time received from Upper is modified to be
 * newer than last timestamp received.
 *
 * @author Thomas Röfer
 */

#include "CognitionVRC.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"

REGISTER_EXECUTION_UNIT(CognitionVRC)

bool CognitionVRC::beforeFrame()
{
  // Currently replaying a log file?
  const bool replay = SystemCall::getMode() == SystemCall::logFileReplay && LogDataProvider::exists();

  // During replay if there is no new frame and no delayed frame, there is nothing to process
  if(replay && !LogDataProvider::isFrameDataComplete(false))
    return false;

  const FrameInfo* lowerFrameInfo = Blackboard::getInstance().exists("LowerFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["LowerFrameInfo"]))
                                    : nullptr;
  FrameInfo* upperFrameInfo = Blackboard::getInstance().exists("UpperFrameInfo")
                              ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["UpperFrameInfo"]))
                              : nullptr;
  unsigned lowerFrameTime = lowerFrameInfo ? lowerFrameInfo->time : 0;
  unsigned upperFrameTime = upperFrameInfo ? upperFrameInfo->time : 0;

  upperIsNew |= upperFrameTime != lastUpperFrameTime;
  lowerIsNew |= lowerFrameTime != lastLowerFrameTime;
  lastUpperFrameTime = upperFrameTime;
  lastLowerFrameTime = lowerFrameTime;

  // During replay, acknowledge that a new frame was received
  if(replay && (upperIsNew || lowerIsNew))
    LogDataProvider::isFrameDataComplete();

  // Prefer Upper, because it is received less often.
  if(upperIsNew)
  {
    upperIsNew = false;
    isUpper = true;

    // Time must move forward, and Upper is probably older than lower
    lastAcceptedTime = std::max(lastUpperFrameTime, lastAcceptedTime + 1);
    lastUpperFrameTime = lastAcceptedTime;
    if(upperFrameInfo)
      upperFrameInfo->time = lastAcceptedTime;
    return true;
  }
  else if(lowerIsNew)
  {
    lowerIsNew = false;
    isUpper = false;

    // Keep original lower timestamp to support rewinding logs
    lastAcceptedTime = lastLowerFrameTime;
    return true;
  }
  else
    return false;
}

bool CognitionVRC::afterFrame()
{
  return !upperIsNew && !lowerIsNew;
}

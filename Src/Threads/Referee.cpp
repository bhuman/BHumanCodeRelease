/**
 * @file Referee.cpp
 *
 * This file implements a thread that handles the referee pose detection.
 *
 * @author Ayleen Lührsen
 */

#include "Referee.h"
#include "Framework/Blackboard.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/Thread.h"

REGISTER_EXECUTION_UNIT(Referee)

bool Referee::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}

bool Referee::afterFrame()
{
  if(Blackboard::getInstance().exists("OptionalCameraImage"))
    return true;
  else
  {
    Thread::yield();
    return false;
  }
}

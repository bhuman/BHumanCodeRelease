/**
 * @file Threads/Perception.cpp
 *
 * This file implements the execution unit for threads that handle the perception.
 *
 * @author Jan Fiedler
 */

#include "Perception.h"
#include "Modules/Infrastructure/CameraProvider/CameraProvider.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"

REGISTER_EXECUTION_UNIT(Perception)

bool Perception::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete();
}

void Perception::beforeModules()
{
  // Reset coordinate system for debug field drawing
  DEBUG_DRAWING("perception:Reset", "drawingOnField") // Set the origin to the (0,0,0)
  {
    ORIGIN("perception:Reset", 0, 0, 0);
  }
}

bool Perception::afterFrame()
{
  if(Blackboard::getInstance().exists("CameraImage"))
  {
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      Thread::getCurrentThread()->setPriority(10);
    Thread::sleep(1);
    BH_TRACE_MSG("before waitForFrameData");
    CameraProvider::waitForFrameData();
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      Thread::getCurrentThread()->setPriority(0);
  }
  else
    Thread::sleep(33);

  return FrameExecutionUnit::afterFrame();
}

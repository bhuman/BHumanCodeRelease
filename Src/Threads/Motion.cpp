/**
 * @file Threads/Motion.cpp
 *
 * This file implements the execution unit for the motion thread.
 *
 * @author Jan Fiedler
 */

#include "Modules/Infrastructure/BoosterProvider/BoosterProvider.h"
#include "Motion.h"
#include "Framework/ModulePacket.h"
#include "Framework/Settings.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Streaming/Global.h"

REGISTER_EXECUTION_UNIT(Motion)

bool Motion::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}

void Motion::afterModules()
{
  BoosterProvider::finishFrame();
}

bool Motion::afterFrame()
{
  if(Blackboard::getInstance().exists("JointSensorData"))
  {
    BH_TRACE_MSG("before waitForFrameData");
    BoosterProvider::waitForFrameData();
  }
  else
    Thread::sleep(static_cast<unsigned>(Global::getSettings().motionCycleTime * 1000.f));

  return BHExecutionUnit::afterFrame();
}

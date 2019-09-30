/**
 * @file Threads/Motion.cpp
 *
 * This file implements the execution unit for the motion thread.
 *
 * @author Jan Fiedler
 */

#include "Modules/Infrastructure/NaoProvider/NaoProvider.h" // include must be the first, because of Visual Studio
#include "Motion.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Tools/Framework/Communication.h"
#include "Tools/Math/Constants.h"
#include "Tools/Module/ModulePacket.h"

REGISTER_EXECUTION_UNIT(Motion)

bool Motion::beforeFrame()
{
  return LogDataProvider::isFrameDataComplete();
}

void Motion::afterModules()
{
  NaoProvider::finishFrame();
}

bool Motion::afterFrame()
{
  if(Blackboard::getInstance().exists("JointSensorData"))
  {
    BH_TRACE_MSG("before waitForFrameData");
    NaoProvider::waitForFrameData();
  }
  else
    Thread::sleep(static_cast<unsigned>(Constants::motionCycleTime * 1000.f));

  return FrameExecutionUnit::afterFrame();
}

#if (defined LINUX || defined MACOS)
#include "Modules/Sensing/FallDownStateDetector/FallDownStateProvider.h"
#include "Modules/Sensing/InertialDataProvider/InertialDataProvider.h"
#include "Modules/MotionControl/WalkingEngine/Walk2014Generator.h"

extern Module<FallDownStateProvider, FallDownStateProviderBase> theFallDownStateProviderModule;
auto linkFallDownStateProvider = &theFallDownStateProviderModule;
extern Module<InertialDataProvider, InertialDataProviderBase> theInertialDataProviderModule;
auto linkInertialDataProvider = &theInertialDataProviderModule;
extern Module<Walk2014Generator, Walk2014GeneratorBase> theWalk2014GeneratorModule;
auto linkWalk2014Generator = &theWalk2014GeneratorModule;
#endif

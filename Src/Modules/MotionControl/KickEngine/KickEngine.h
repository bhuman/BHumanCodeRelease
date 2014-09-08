/**
* @file KickEngine.h
* This file declares a module that creates the walking motions.
* @author <A href="mailto:judy@tzi.de">Judith Müller</A>
*/

#pragma once

#ifndef WINDOWS
#include <dirent.h>
#else
#include <windows.h>
#endif
#include <cstdio>
#include <cstring>

#include "KickEngineParameters.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"
#include "KickEngineData.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Platform/File.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(KickEngine,
{,
  REQUIRES(RobotDimensions),
  REQUIRES(JointCalibration),
  REQUIRES(FrameInfo),
  REQUIRES(FilteredSensorData),
  REQUIRES(MotionSelection),
  REQUIRES(MotionRequest),
  REQUIRES(FilteredJointData),
  REQUIRES(MassCalibration),
  REQUIRES(RobotModel),
  REQUIRES(WalkingEngineStandOutput),
  REQUIRES(TorsoMatrix),
  REQUIRES(HeadJointRequest),
  PROVIDES_WITH_MODIFY(KickEngineOutput),
});

class KickEngine : public KickEngineBase
{
private:

  KickEngineData data;
  bool compensate, compensated;
  unsigned timeSinceLastPhase;

  std::vector<KickEngineParameters> params;

public:

  void update(KickEngineOutput& kickEngineOutput);

  /**
  * Default constructor.
  */
  KickEngine();
};

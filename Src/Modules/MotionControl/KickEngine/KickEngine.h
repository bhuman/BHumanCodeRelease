/**
 * @file KickEngine.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

#include "KickEngineData.h"
#include "KickEngineParameters.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"

MODULE(KickEngine,
{,
  REQUIRES(FrameInfo),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointCalibration),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(MotionSelection),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(StandOutput),
  PROVIDES(KickEngineOutput),
});

class KickEngine : public KickEngineBase
{
private:
  KickEngineData data;
  bool compensate, compensated;
  unsigned timeSinceLastPhase;

  std::vector<KickEngineParameters> params;

public:
  KickEngine();

  void update(KickEngineOutput& kickEngineOutput);
};

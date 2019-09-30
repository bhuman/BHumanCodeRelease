/**
 * @file KickEngine.h
 * This file declares a module that creates the walking motions.
 * @author <A href="mailto:judy@tzi.de">Judith Müller</A>
 */

#pragma once

#include "KickEngineData.h"
#include "KickEngineParameters.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/KickEngineOutput.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"
#include "Tools/Streams/InStreams.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

MODULE(KickEngine,
{,
  USES(JointRequest),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(JointLimits),
  REQUIRES(MassCalibration),
  REQUIRES(MotionRequest),
  REQUIRES(LegMotionSelection),
  REQUIRES(KeyStates),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(KickEngineOutput),
});

class KickEngine : public KickEngineBase
{
private:
  KickEngineData data;
  bool compensate = false;
  bool compensated = false;
  int boostState = 0;
  unsigned timeSinceLastPhase = 0;
  KickRequest lastValidKickRequest;

  std::vector<KickEngineParameters> params;

public:
  KickEngine();

  void update(KickEngineOutput& kickEngineOutput) override;
};

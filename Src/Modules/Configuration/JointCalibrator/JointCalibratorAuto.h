/**
 * @file Modules/Infrastructure/JointCalibratorAuto.h
 * This file declares a module that calibrates the joint automatic
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Tools/Module/Module.h"
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Motion/MotionUtilities.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/Debugging/DebugDrawings.h"

STREAMABLE(PhasePairs,
{,
  (Joints::Joint) joint,
  (Angle) position,
});

STREAMABLE(Phases,
{,
  (std::vector<PhasePairs>) jointPositions,
  (std::vector<Joints::Joint>) measure,
  (Vector2a) torsoAngle,
});

MODULE(JointCalibratorAuto,
{,
  PROVIDES(JointCalibration),
  REQUIRES(JointCalibration),
  USES(FrameInfo),
  USES(JointAngles),
  USES(JointLimits),
  USES(InertialData),
  PROVIDES(JointRequest),
  LOADS_PARAMETERS(
  {,
    (ENUM_INDEXED_ARRAY(Angle, Joints::Joint)) calibrationReference,
    (Angle) headYawOtherRef,
    (std::vector<Phases>) calibrationPoses,
    (Angle) maxTorsoDif,
    (int) responseWaitTime,
    (int) switchTransitionWaitTime,
    (int) samplingTime,
    (float) switchInterpolation,
    (int) stiffnessArms,
    (int) stiffnessHead,
    (int) stiffnessLegs,
    (int) stiffnessMeasuredJoints,
  }),
});

class JointCalibratorAuto : public JointCalibratorAutoBase
{
public:

  void update(JointCalibration& jointCalibration) override;

  //This update method is not needed anymore, when a Call Scene is added.
  void update(JointRequest& jointRequest) override;

  JointCalibratorAuto();

private:

  ENUM(State,
  {,
    off,
    switching,
    sampling,
    waiting,
    finished,
  });

  STREAMABLE(Sample,
  {,
    (Joints::Joint) joint,
    (Angle) min,
    (Angle) max,
  });

  std::vector<Sample> sample;
  JointRequest startJoints;
  JointRequest targetJoints;
  unsigned int waitingResponse;
  unsigned int switchTransition;
  int index;
  State calibrationState;
  ENUM_INDEXED_ARRAY(Vector3a, Joints::Joint) measuredPositions;
  JointCalibration calibration;

  void calculateNextTarget();
};

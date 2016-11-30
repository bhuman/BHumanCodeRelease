/**
 * @file WalkingEngineLegProvider.h
 *
 * @author Colin Graf
 * @author Felix Wenk
 * @author Alexis Tsogias
 */

#pragma once

#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadJointRequest.h"
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/StandBodyRotation.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkingEngineState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

MODULE(WalkingEngineLegProvider,
{,
  REQUIRES(ArmJointRequest),
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(HeadJointRequest),
  REQUIRES(InertialData),
  REQUIRES(JointAngles),
  REQUIRES(LegMotionSelection),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(StandBodyRotation),
  REQUIRES(WalkingEngineState),

  PROVIDES(WalkingEngineOutput),
  REQUIRES(WalkingEngineOutput),

  PROVIDES(WalkLegRequest),
  PROVIDES(StandLegRequest),
  LOADS_PARAMETERS(
  {,
    (int) anklePitchStiffness, /**< The stiffness of the ankle pitch joint */
    (int) ankleRollStiffness, /**< The stiffness of the ankle roll joint */
    (Angle) walkComBodyRotation, /**< How much the torso is rotated to achieve the center of mass shift along the y-axis */

    // Parameters to calculate the correction of the torso's angular velocity.
    (float) gyroStateGain, /**< Control weight (P) of the torso's angular velocity error. */
    (float) gyroDerivativeGain, /**< Control weight (D) of the approximated rate of change of the angular velocity error. */
    (float) gyroSmoothing, /**< Smoothing (between 0 and 1!) to calculate the moving average of the y-axis gyro measurements. */
  }),
});

class WalkingEngineLegProvider : public WalkingEngineLegProviderBase
{
  Vector3f bodyToCom = Vector3f::Zero();
  Vector3f lastAverageComToAnkle = Vector3f::Zero();
  RotationMatrix lastBodyRotationMatrix;
  RingBuffer<float, 10> relativeRotations;
  float lastSmoothedGyroY = 0.f; /**< Moving average of the y-axis gyro from the previous motion frame. */
  float lastGyroErrorY = 0.f; /**< Last y-axis gyro deviation from the commanded angular velocity of the torso. */

  LegJointRequest legJointRequest;

  void update(WalkingEngineOutput& walkingEngineOutput) override;

  void update(WalkLegRequest& walkLegRequest) override { static_cast<LegJointRequest&>(walkLegRequest) = legJointRequest; }
  void update(StandLegRequest& standLegRequest) override { static_cast<LegJointRequest&>(standLegRequest) = legJointRequest; }

  void generateLegJointRequest();
  void generateWalkingEngineOutput(WalkingEngineOutput& walkingEngineOutput);
};

/**
 * @file FilteredCurrentProvider.h
 * This module filters the currents of the joints and checks for motor malfunctions.
 * A motor malfunction is a motor of a joint, which is not working anymore or stopped working for a moment.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/Sensing/GyroOffset.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Sensing/FilteredCurrent.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Debugging/Annotation.h"
#include "Framework/Module.h"
#include "Math/RingBufferWithSum.h"
#include "Debugging/DebugDrawings.h"
#include "Streaming/EnumIndexedArray.h"

MODULE(FilteredCurrentProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GroundContactState),
  REQUIRES(GyroOffset),
  REQUIRES(JointAngles),
  USES(JointRequest),
  REQUIRES(JointSensorData),
  USES(MotionInfo),
  PROVIDES(FilteredCurrent),
  LOADS_PARAMETERS(
  {,
    (int) stiffnessThreshold, /**< Min stiffness for the legs to check for a motor malfunction. */
    (int) stiffnessThresholdArms, /**< Min stiffness for the arms to check for a motor malfunction. */
    (int) flagsThreshold, /**< Min counter for a detected motor malfunction. */
    (int) checkWaitTime, /**< Wait time between motor malfunction checks. */
    (int) motorMalfunctionTime, /**< Sound wait time for a detected motor malfunction. */
    (int) annotationWaitTime, /**< Wait time for motor malfunction annotations. */
    (Angle) minJointDiffNormalJoints, /**< Min difference in jointRequest and jointAngles for the legs, to detect a motor malfunction. */
    (Angle) minJointDiffNormalJointsNoGroundContact, /**< Min difference in jointRequest and jointAngles when no ground contact is detected, to detect a motor malfunction. */
    (Angle) minJointDiffAnkleRoll, /**< Min difference in jointRequest and jointAngles for the ankleRolls, to detect a motor malfunction. This value must be high, because the current can be 0 at high differences. */
    (Angle) minJointDiffArms, /**< Min difference in jointRequest and jointAngles for the arms, to detect a motor malfunction. */
    (std::vector<Joints::Joint>) ignoreJoints, /**< Joints which are not checks for a motor malfunction. */
    (std::vector<Joints::Joint>) specialMotorCheck,
    (Angle) zeroCheckRange,
    (int) filterSize,
  }),
});

class FilteredCurrentProvider : public FilteredCurrentProviderBase
{
public:
  FilteredCurrentProvider();
private:

  std::vector<RingBufferWithSum<int, 20>> currents; /**< Ring buffer for the currents for every joint. */
  unsigned int checkTimestamp = 0; /**< Last time a motor malfunction was checked. */
  unsigned int soundTimestamp = 0; /**< Last time a sound was played for a motor malfunction. */
  unsigned int annotationTimestamp = 0; /**< Last time a annotation was made for a motor malfunction. */
  std::vector<int> flags; /**< Checks how often a possible motor malfunction was detected. */

  void update(FilteredCurrent& theFilteredCurrent) override;
  void checkMotorMalfunction(FilteredCurrent& theFilteredCurrent);
};

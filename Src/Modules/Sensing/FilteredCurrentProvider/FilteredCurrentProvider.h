/**
 * @file FilteredCurrentProvider.h
 * This module filters the currents of the joints and checks for motor malfunctions.
 * A motor malfunction is a motor of a joint, which is not working anymore or stopped working for a moment.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Configuration/GyroOffset.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Sensing/FilteredCurrent.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBufferWithSum.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Streams/EnumIndexedArray.h"

MODULE(FilteredCurrentProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(GyroOffset),
  REQUIRES(RobotInfo),
  USES(JointRequest),
  REQUIRES(JointSensorData),
  PROVIDES(FilteredCurrent),
  DEFINES_PARAMETERS(
  {,
    (int)(60) stiffnessThreshold, /**< Min stiffness for the legs to check for a motor malfunction. */
    (int)(40) stiffnessThresholdArms, /**< Min stiffness for the arms to check for a motor malfunction. */
    (int)(5) flagsThreshold, /**< Min counter for a detected motor malfunction. */
    (int)(300) checkWaitTime, /**< Wait time between motor malfunction checks. */
    (int)(3000) motorMalfunctionTime, /**< Sound wait time for a detected motor malfunction. */
    (int)(45000) annotationWaitTime, /**< Wait time for motor malfunction annotations. */
    (Angle)(4_deg) minJointDifNormalJoints, /**< Min difference in jointRequest and jointAngles for the legs, to detect a motor malfunction. */
    (Angle)(8_deg) minJointDifNormalJointsNoGroundConntact, /**< Min difference in jointRequest and jointAngles when no ground contact is detected, to detect a motor malfunction. */
    (Angle)(10_deg) minJointDifAnkleRoll, /**< Min difference in jointRequest and jointAngles for the ankleRolls, to detect a motor malfunction. This value must be high, because the current can be 0 at high differences. */
    (Angle)(6_deg) minJointDifArms, /**< Min difference in jointRequest and jointAngles for the arms, to detect a motor malfunction. */
    (std::vector<Joints::Joint>)({Joints::headYaw, Joints::headPitch, Joints::lWristYaw, Joints::rWristYaw, Joints::lHand, Joints::rHand}) ignoreJoints, /**< Joints which are not checks for a motor malfunction. */
  }),
});

class FilteredCurrentProvider : public FilteredCurrentProviderBase
{
public:
  FilteredCurrentProvider();
private:

  std::vector<RingBufferWithSum<int, 20>> currents; /**< Ring buffer for the currents for every joint. */
  unsigned int checkTimestamp; /**< Last time a motor malfunction was checked. */
  unsigned int soundTimestamp; /**< Last time a sound was played for a motor malfunction. */
  unsigned int annotationTimestamp; /**< Last time a annotation was made for a motor malfunction. */
  std::vector<int> flags; /**< Checks how often a possible motor malfunction was detected. */

  void update(FilteredCurrent& theFilteredCurrent) override;
  void checkMotorMalfunction(FilteredCurrent& theFilteredCurrent);
};

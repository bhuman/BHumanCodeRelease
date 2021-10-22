/**
 * @file FilteredCurrentProvider.h
 * This module provides offsets for the current jointRequest, to ensure that the currents on the joints are as low a possible, so they do not heat up.
 * @author Philip Reichenberg
 */

#pragma once

#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/EnergySaving.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FilteredCurrent.h"
#include "Tools/Module/Module.h"

using EnergyState = EnergySaving::EnergyState;

MODULE(EnergySavingProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FilteredCurrent),
  REQUIRES(GroundContactState),
  REQUIRES(JointAngles),
  PROVIDES(EnergySaving),
  LOADS_PARAMETERS(
  {,
    (int) motionChangeWaitTime, /**< If we are in a motion for the first time, where we can adjust the joints, we want to wait this long. */
    (int) adjustWaitTime, /**< After every adjustment, wait this long. */
    (int) currentThresholdLegs, /**< If the current is below this threshold, we do not want to adjust the leg joints. */
    (int) currentThresholdArms, /**< If the current is below this threshold, we do not want to adjust the arm joints. */
    (Angle) minGearStep, /**< The minimal step a gear can make that can be measured (in radians). */
    (Angle) maxGearStepArms, /**< If we can adjust in a bigger chunks for the arms .*/
    (Angle) maxGearStepLegs, /**< If we can adjust in a bigger chunks for the legs .*/
    (Angle) maxAngleMultipleJoints, /**< The maximal adjustment for the joints when the robot is standing, to insure that the robot will not fall after some time. */
    (Angle) maxAngleOneJoint, /**< The maximal adjustment for one joint when the robot is standing. If this threshold is exceeded, the adjustments are reset for all leg joints. */
    (int) minNumberForHeatAdjustmentReset, /** Min number of joints that must have an adjustment equal to maxJointHeatAdjustment to reset all adjustments .*/
    (float) resetTimeNormal, /**< Time to reduce the offsets back to 0. */
    (float) resetTimeSlow, /**< Time to reduce the offsets back to 0. */
    (int) stepsBeforeEmergencyStep, /**< After so many steps, where the current is still too high, we want to adjust by a large amount, to make sure the current get reduced after sometime.
                                          The big step is the negative value of maxGearStep. */
    (std::vector<Joints::Joint>) skipJoints, /**< Do not adjust these joints. HeadPitch and HeadYaw are assumed to be ignored by the motion engines. */
    (ENUM_INDEXED_ARRAY(int, Joints::Joint)) highStandSigns, /**< emergencyStep with maxGearStep with this sign in highStand. */
    (ENUM_INDEXED_ARRAY(int, Joints::Joint)) standSigns, /**< emergencyStep with maxGearStep with this sign in Stand. */
  }),
});

class EnergySavingProvider : public EnergySavingProviderBase
{
public:
  EnergySavingProvider();

private:

  ENUM(State,
  {,
    deactive,
    energySaving,
    accuratePosition,
  });

  ENUM_INDEXED_ARRAY(Angle, Joints::Joint) lastOffsets;
  ENUM_INDEXED_ARRAY(int, Joints::Joint) emergencyChangeCounter;
  unsigned int motionChangeTimestamp = 0; /**< Time to wait until adjusting the leg joints to prevent overheating. */
  unsigned int adjustTimestamp = 0; /**< Time to wait after an adjustment. */
  unsigned int lastInAdjustmentTimestamp = 0;
  bool hasGroundContact = false;
  State state = State::deactive;
  State lastState = State::deactive;
  void update(EnergySaving& energySaving) override;
  float usedResetInterpolation = resetTimeNormal;
};

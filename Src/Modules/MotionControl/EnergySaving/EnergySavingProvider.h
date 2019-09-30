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
#include "Representations/MotionControl/LegMotionSelection.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/FilteredCurrent.h"
#include "Tools/Module/Module.h"

MODULE(EnergySavingProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FilteredCurrent),
  REQUIRES(GroundContactState),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(LegMotionSelection),
  REQUIRES(MotionRequest),
  PROVIDES(EnergySaving),
  LOADS_PARAMETERS(
  {,
    (int) motionChangeWaitTime, /**< If we are in a motion for the first time, where we can adjust the joints, we want to wait this long. */
    (int) adjustWaitTime, /**< After every adjustment, wait this long. */
    (int) currentThresholdLegs, /**< If the current is below this threshold, we dont want to adjust the leg joints. */
    (int) currentThresholdArms, /**< If the current is below this threshold, we dont want to adjust the arm joints. */
    (Angle) minGearStep, /**< The minimal step a gear can make that can be measured (in radians). */
    (Angle) maxGearStepArms, /**< If we can adjust in a bigger chunks for the arms .*/
    (Angle) maxGearStepLegs, /**< If we can adjust in a bigger chunks for the legs .*/
    (Angle) maxAngleMultipleJoints, /**< The maximal adjustment for the joints when the robot is standing, to insure that the robot will not fall after some time. */
    (Angle) maxAngleOneJoint, /**< The maximal adjustment for one joint when the robot is standing. If this threshold is exceeded, the adjustments are resetet for all leg joints. */
    (int) minNumberForHeatAdjustmentReset, /** Min number of joints that must have an adjustment equal to maxJointHeatAdjustment to reset all adjustments .*/
    (float) resetTime, /**< Time to reduce the offsets back to 0. */
    (int) stepsBeforeEmergencyStep, /**< After so many steps, where the current is still too high, we want to adjust by a large amount, to make sure the current get reduced after sometime.
                                          The big step is the negative value of maxGearStep. */
    (std::vector<Joints::Joint>) skipJoints, /**< Do not adjust these joints. */
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
    off,
    waiting,
    working,
    reset,
  });
  State state;
  ENUM_INDEXED_ARRAY(Angle, Joints::Joint) lastOffsets;
  ENUM_INDEXED_ARRAY(int, Joints::Joint) emergencyChangeCounter;
  unsigned int motionChangeTimestamp; /**< Time to wait until adjusting the leg joints to prevent overheating. */
  unsigned int adjustTimestamp; /**< Time to wait after a adjustment. */
  unsigned int lastInAdjustmentTimestamp;
  bool hasGroundContact;
  bool standHeatInSpecialAction; /**< Is the energy saving mode active in the current SpecialAction. */
  void update(EnergySaving& energySaving) override;
  bool shouldBeActive();
  bool allowedToAdjustArms();
};

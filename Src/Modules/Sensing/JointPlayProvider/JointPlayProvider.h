/**
 * @file JointPlayProvider.h
 * This modules analyzes the joint play and returns information about how bad the hardware state is.
 *
 * The values are determined by calculating the difference between the measured and requested positions
 * (3 frames previous because of the delay).
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/JointPlay.h"
#include "Math/RingBuffer.h"

STREAMABLE(JointPlayPair,
{,
  (Joints::Joint) joint,
  (Angle) ref, // Max joint play values, based on a good robot
  (float) ratio,
});

MODULE(JointPlayProvider,
{,
  REQUIRES(DamageConfigurationBody),
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(JointAngles),
  USES(JointRequest),
  USES(MotionInfo),
  REQUIRES(MotionRequest),
  USES(WalkStepData),
  PROVIDES(JointPlay),
  LOADS_PARAMETERS(
  {,
    (Rangef) lowpassFilterFactor, /**< Low pass filter value */
    (float) interpolateLowpassFilterTime, /**< Start with high filter factor, but interpolate over 5 secs to the lower value (in sec). */
    (float) interpolateJointPlayValueWalkTime, /**< The robot needs to walk for a bit, to allow higher hardware values (in sec). */
    (float) minWalkTime, /**< Start filtering after this much time, after robot started to walk. */
    (std::vector<JointPlayPair>) jointPlayList, /**< TODO. */
    (Angle) jointPlayScalingWalkingSpeed, /**< When walking slower, a lower joint play is expected. */
    (float) maxForwardSpeed, /**< Scale expected joint play until this max speed. */
    (float) minForwardSpeed, /**< Scale expected joint play from this min speed. */
    (Rangea) jointPlayScaling, /**< A robot below the minimum is good, a robot above bad. */
    (int) warningSoundWaitTime,
    (Angle) minPlayForSound,
    (Angle) minPlaySlowForSound,
    (float) numOfFramesSkip, /**< Sometime a motion frame is missing. We allow a few skips, before resetting some timers. */
    (int) motionDelay,
  }),
});

class JointPlayProvider : public JointPlayProviderBase
{
  void update(JointPlay& theJointPlay) override;

  // Buffered requests, so we can compare the actually executed one with the measured angles
  RingBuffer<Angle> bufferRequest[Joints::numOfJoints];

  // Filtered values over a long period of time.
  Angle bufferValue[Joints::numOfJoints];

  // last measured angles
  JointAngles lastJointAngles;

  RingBuffer<Angle, 2> jointPlayValue[Joints::numOfJoints];

  // Timestamp walking started
  unsigned int startWalkingTimestamp = 0;

  // Time spend walking
  float timeSpendWalking = 0.f;

  // Is robot currently walking?
  bool isWalking = false;

  // We use USES(), therefore skip the first data
  bool skippedFirstFrame = false;

  // When was the last warning?
  unsigned int lastWarningTimestamp = 0;

  // What was the previous FrameInfo time?
  // Assign largest number. Otherwise no detection is possible in the first few seconds after the software started
  unsigned int lastFrameInfoTime = std::numeric_limits<unsigned int>::max();

public:
  JointPlayProvider();
};

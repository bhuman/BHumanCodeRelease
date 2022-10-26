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
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/JointPlay.h"
#include "Math/RingBufferWithSum.h"

// Enum for the joints to be tracked
ENUM(JointPlayTrack,
{,
  lhp,
  lkp,
  lap,
  rhp,
  rkp,
  rap,
});

MODULE(JointPlayProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  REQUIRES(WalkStepData),
  PROVIDES(JointPlay),
  LOADS_PARAMETERS(
  {,
    (Rangef) lowpassFilterFactor, /**< Low pass filter value */
    (float) interpolateLowpassFilterTime, /**< Start with high filter factor, but interpolate over 5 secs to the lower value. */
    (float) minWalkTime, /**< Start filtering after this much time, after robot started to walk. */
    (ENUM_INDEXED_ARRAY(Angle, JointPlayTrack)) maxJointPlay, /**< Max joint play values, based on a good robot. */
    (ENUM_INDEXED_ARRAY(float, JointPlayTrack)) maxJointPlayRatio, /**< Max joint play values, based on a good robot. */
    (Angle) jointPlayScalingWalkingSpeed, /**< When walking slower, a lower joint play is expected. */
    (float) maxForwardSpeed, /**< Scale expected joint play until this max speed. */
    (float) minForwardSpeed, /**< Scale expected joint play from this min speed. */
    (Rangea) jointPlayScaling, /**< A robot below the minimum is good, a robot above bad. */
  }),
});

class JointPlayProvider : public JointPlayProviderBase
{
  void update(JointPlay& theJointPlay) override;

  // Converts JointPlayTrack into the Joint enum
  Joints::Joint getJoint(JointPlayTrack joint);

  // Buffer for the joint request. Needed because of the motion delay, until a request is executed
  RingBufferWithSum<Angle, 4> bufferRequest[JointPlayTrack::numOfJointPlayTracks];

  // Filtered values over a long periode of time.
  Angle bufferValue[JointPlayTrack::numOfJointPlayTracks];

  // Filtered values over a long periode of time with a less strong low-pass filter parameter
  // Currently only used to analyse by hand. Shall be used in the future
  // to check for high changes within a few frames. Those indicate damaged gears/joints
  Angle bufferValueShortTerm[JointPlayTrack::numOfJointPlayTracks];

  // Temp buffer for the joint play, relative to a good robot
  Angle jointPlayList[JointPlayTrack::numOfJointPlayTracks];

  // Timestamp walking started
  unsigned int startWalkingTimestamp = 0;

  // Time spend walking
  float timeSpendWalking = 0.f;

  // Is robot currently walking?
  bool isWalking = false;

public:
  JointPlayProvider();
};

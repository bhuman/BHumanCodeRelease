/**
 * @file MotionPhase.h
 *
 * This file declares a base struct for phases.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Enum.h"
#include <memory>

struct MotionInfo;
struct MotionRequest;

struct MotionPhase
{
  ENUM(Type,
  {,
    playDead,
    stand,
    walk,
    kick,
    fall,
    getUp,
    keyframeMotion,
    replayWalk,
  });

  ENUM(Limb,
  {,
    head,
    leftArm,
    rightArm,
  });

  /**
   * Constructor.
   * @param type The type of the phase.
   */
  explicit MotionPhase(Type type) :
    type(type)
  {}

  /** Virtual destructor for polymorphism. */
  virtual ~MotionPhase() = default;

  /** Updates the state of the phase. */
  virtual void update()
  {}

  /**
   * Returns whether the phase should be left.
   * @param motionRequest The motion request.
   * @return Whether the phase should be left.
   */
  virtual bool isDone(const MotionRequest& motionRequest) const = 0;

  /**
   * Creates the joint request for the current frame of the phase.
   * @param motionRequest The motion request.
   * @param jointRequest The joint request that is filled.
   * @param odometryOffset The odometry offset that is filled for this frame (is zero prior to this method).
   * @param motionInfo The motion info that is filled (phase type is already set).
   */
  virtual void calcJoints(const MotionRequest& motionRequest, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo) = 0;

  /**
   * Returns a phase that must follow the current one.
   * @param defaultNextPhase The phase that would follow otherwise.
   * @return The next phase.
   */
  virtual std::unique_ptr<MotionPhase> createNextPhase(const MotionPhase& defaultNextPhase) const;

  /**
   * Returns the bit set of limbs that are not necessarily controlled by this phase.
   * @return A logical or of limbs that can be controlled by other engines.
   */
  virtual unsigned freeLimbs() const
  {
    return 0;
  }

  Type type; /**< The type of this phase. */
  unsigned kickType; /**< The type of kick in this phase (only valid if it sets \c motionInfo.isKicking()). Cannot be \c KickInfo::KickType due to circular include dependencies. */
};

inline std::unique_ptr<MotionPhase> MotionPhase::createNextPhase(const MotionPhase&) const
{
  return std::unique_ptr<MotionPhase>();
}

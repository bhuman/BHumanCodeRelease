/**
 * @file ArmObstacleAvoidance.cpp
 *
 * This file implements implementations of the ArmObstacleAvoidance
 * and ArmObstacleAvoidanceSingleArm skills.
 *
 * @author Arne Hasselbring
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"

SKILL_IMPLEMENTATION(ArmObstacleAvoidanceImpl,
{,
  IMPLEMENTS(ArmObstacleAvoidance),
  IMPLEMENTS(ArmObstacleAvoidanceSingleArm),
  CALLS(KeyFrameSingleArm),
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  USES(MotionRequest),
  REQUIRES(ObstacleModel),
  LOADS_PARAMETERS(
  {,
    (int) stayBackTime, /**< The duration for which the arm should stay back after the obstacle is not in the range anymore. */
    (int) deadTime, /**< The duration for which an arm should not be taken back after it went back to normal mode. */
    (float) distance, /**< Only obstacles closer than this are considered. */
  }),
});

class ArmObstacleAvoidanceImpl : public ArmObstacleAvoidanceImplBase
{
  void execute(const ArmObstacleAvoidance&) override
  {
    setRequest(Arms::left);
    setRequest(Arms::right);
  }

  void reset(const ArmObstacleAvoidance&) override
  {
    timesWhenShouldBeBack[Arms::left] = timesWhenShouldBeBack[Arms::right] = 0;
    timesWhenActionEnded[Arms::left] = timesWhenActionEnded[Arms::right] = theFrameInfo.time;
  }

  void execute(const ArmObstacleAvoidanceSingleArm& p) override
  {
    setRequest(p.arm);
  }

  void reset(const ArmObstacleAvoidanceSingleArm& p) override
  {
    timesWhenShouldBeBack[p.arm] = 0;
    timesWhenActionEnded[p.arm] = theFrameInfo.time;
  }

  void setRequest(Arms::Arm arm)
  {
    if(theFallDownState.state != FallDownState::upright)
      timesWhenShouldBeBack[arm] = 0;
    else if(shouldBeBack(arm) && theFrameInfo.getTimeSince(timesWhenActionEnded[arm]) >= deadTime)
      timesWhenShouldBeBack[arm] = theFrameInfo.time;
    else if(!theMotionRequest.isWalking())
      timesWhenShouldBeBack[arm] = 0;

    if(theFrameInfo.getTimeSince(timesWhenShouldBeBack[arm]) < stayBackTime)
    {
      theKeyFrameSingleArmSkill(ArmKeyFrameRequest::back, arm);
      timesWhenActionEnded[arm] = 0;
    }
    else if(timesWhenActionEnded[arm] == 0)
      timesWhenActionEnded[arm] = theFrameInfo.time;
  }

  bool shouldBeBack(Arms::Arm arm) const
  {
    if(theMotionInfo.executedPhase != MotionPhase::walk || theFallDownState.state != FallDownState::upright)
      return false;

    const float distanceSquared = sqr(distance);
    // This sign makes the inequality in which it is used applicable for left as well as right.
    const float sign = arm == Arms::left ? 1.f : -1.f;
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      if((obstacle.left.squaredNorm() < distanceSquared && sign * obstacle.left.angle() > pi_8) ||
         (obstacle.center.squaredNorm() < distanceSquared && sign * obstacle.center.angle() > pi_8) ||
         (obstacle.right.squaredNorm() < distanceSquared && sign * obstacle.right.angle() > pi_8))
        return true;

    return false;
  }

  unsigned timesWhenShouldBeBack[Arms::numOfArms]; /**< The last time when an arm should be taken back to avoid an obstacle. */
  unsigned timesWhenActionEnded[Arms::numOfArms]; /**< The last time when an arm action by this skill ended. */
};

MAKE_SKILL_IMPLEMENTATION(ArmObstacleAvoidanceImpl);

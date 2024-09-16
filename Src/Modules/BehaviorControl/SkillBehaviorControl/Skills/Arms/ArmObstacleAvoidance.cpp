/**
 * @file ArmObstacleAvoidance.cpp
 *
 * This file implements the ArmObstacleAvoidance skills.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) ArmObstacleAvoidance,
       args((Arms::Arm) arm))
{
  common_transition
  {
    if(arm == Arms::numOfArms)
      goto both;
    else if(arm == Arms::left)
      goto left;
    else
      goto right;
  }

  initial_state(both)
  {
    action
    {
      ArmObstacleAvoidanceLeftArm();
      ArmObstacleAvoidanceRightArm();
    }
  }

  state(left)
  {
    action
    {
      ArmObstacleAvoidanceLeftArm();
    }
  }

  state(right)
  {
    action
    {
      ArmObstacleAvoidanceRightArm();
    }
  }
}

// no #pragma once

option((SkillBehaviorControl) ArmObstacleAvoidanceLeftArm,
       defs((int)(1000) stayBackTime, /**< The duration for which the arm should stay back after the obstacle is not in the range anymore. */
            (int)(500) deadTime, /**< The duration for which an arm should not be taken back after it went back to normal mode. */
            (float)(400.f) distance)) /**< Only obstacles closer than this are considered. */
{
  const auto shouldBeBack = [&]
  {
    if(theMotionInfo.executedPhase != MotionPhase::walk || theFallDownState.state != FallDownState::upright)
      return false;

    const float distanceSquared = sqr(distance);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      if((obstacle.left.squaredNorm() < distanceSquared && obstacle.left.angle() > pi_8) ||
         (obstacle.center.squaredNorm() < distanceSquared && obstacle.center.angle() > pi_8) ||
         (obstacle.right.squaredNorm() < distanceSquared && obstacle.right.angle() > pi_8))
        return true;

    return false;
  };

  initial_state(armNormal)
  {
    transition
    {
      if(theFallDownState.state == FallDownState::upright && shouldBeBack())
        goto armBack;
    }
  }

  state(armBack)
  {
    transition
    {
      if(theFallDownState.state != FallDownState::upright)
        goto delay;
      else if(!shouldBeBack())
        goto keepBack;
    }
    action
    {
      KeyFrameLeftArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(keepBack)
  {
    transition
    {
      if(shouldBeBack())
        goto armBack;
      else if(state_time > stayBackTime)
        goto delay;
    }
    action
    {
      KeyFrameLeftArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(delay)
  {
    transition
    {
      if(!theMotionRequest.isWalking() || state_time > deadTime)
        goto armNormal;
    }
  }
}

option((SkillBehaviorControl) ArmObstacleAvoidanceRightArm,
       defs((int)(1000) stayBackTime, /**< The duration for which the arm should stay back after the obstacle is not in the range anymore. */
            (int)(500) deadTime, /**< The duration for which an arm should not be taken back after it went back to normal mode. */
            (float)(400.f) distance)) /**< Only obstacles closer than this are considered. */
{
  const auto shouldBeBack = [&]
  {
    if(theMotionInfo.executedPhase != MotionPhase::walk || theFallDownState.state != FallDownState::upright)
      return false;

    const float distanceSquared = sqr(distance);
    for(const Obstacle& obstacle : theObstacleModel.obstacles)
      if((obstacle.left.squaredNorm() < distanceSquared && -obstacle.left.angle() > pi_8) ||
         (obstacle.center.squaredNorm() < distanceSquared && -obstacle.center.angle() > pi_8) ||
         (obstacle.right.squaredNorm() < distanceSquared && -obstacle.right.angle() > pi_8))
        return true;

    return false;
  };

  initial_state(armNormal)
  {
    transition
    {
      if(theFallDownState.state == FallDownState::upright && shouldBeBack())
        goto armBack;
    }
  }

  state(armBack)
  {
    transition
    {
      if(theFallDownState.state != FallDownState::upright)
        goto delay;
      else if(!shouldBeBack())
        goto keepBack;
    }
    action
    {
      KeyFrameRightArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(keepBack)
  {
    transition
    {
      if(shouldBeBack())
        goto armBack;
      else if(state_time > stayBackTime)
        goto delay;
    }
    action
    {
      KeyFrameRightArm({.motion = ArmKeyFrameRequest::back});
    }
  }

  state(delay)
  {
    transition
    {
      if(!theMotionRequest.isWalking() || state_time > deadTime)
        goto armNormal;
    }
  }
}

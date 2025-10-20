/**
 * @file WalkToPose.cpp
 *
 * This file implements the WalkToPose skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) WalkToPose,
       args((const Pose2f&) target,
            (const Pose2f&) speed,
            (const MotionRequest::ObstacleAvoidance&) obstacleAvoidance,
            (bool) keepTargetRotation,
            (const std::optional<Vector2f>&) targetOfInterest,
            (SideWalkingRequest::SideWalkingRequest) sideWalkingRequest,
            (bool) energySavingWalk))
{
  theMotionRequest.motion = MotionRequest::walkToPose;
  theMotionRequest.walkTarget = target;
  theMotionRequest.walkSpeed = speed;
  theMotionRequest.keepTargetRotation = keepTargetRotation;
  theMotionRequest.obstacleAvoidance = obstacleAvoidance;
  theMotionRequest.targetOfInterest = targetOfInterest;
  theMotionRequest.sideWalkingRequest = sideWalkingRequest;
  theMotionRequest.energySavingWalk = energySavingWalk;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::walk)
        goto done;
    }
  }

  target_state(done) {}
}

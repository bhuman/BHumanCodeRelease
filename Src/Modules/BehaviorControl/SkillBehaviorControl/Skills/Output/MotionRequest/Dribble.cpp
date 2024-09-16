/**
 * @file Dribble.cpp
 *
 * This file implements the Dribble skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Dribble,
       args((Angle) targetDirection,
            (const Pose2f&) speed,
            (const MotionRequest::ObstacleAvoidance&) obstacleAvoidance,
            (KickPrecision) alignPrecisely,
            (float) kickLength,
            (PreStepType) preStepType,
            (bool) turnKickAllowed,
            (const Rangea&) directionPrecision))
{
  theMotionRequest.motion = MotionRequest::dribble;
  theMotionRequest.walkSpeed = speed;
  theMotionRequest.obstacleAvoidance = obstacleAvoidance;
  theMotionRequest.targetDirection = targetDirection;
  theMotionRequest.directionPrecision = directionPrecision;
  theMotionRequest.alignPrecisely = alignPrecisely;
  theMotionRequest.kickLength = kickLength;
  theMotionRequest.turnKickAllowed = turnKickAllowed;
  theMotionRequest.preStepType = preStepType;
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

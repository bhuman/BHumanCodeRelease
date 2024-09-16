/**
 * @file WalkToBallAndKick.cpp
 *
 * This file implements the WalkToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) WalkToBallAndKick,
       args((Angle) targetDirection,
            (KickInfo::KickType) kickType,
            (KickPrecision) alignPrecisely,
            (float) kickLength,
            (const Pose2f&) speed,
            (const MotionRequest::ObstacleAvoidance&) obstacleAvoidance,
            (PreStepType)preStepType,
            (bool) turnKickAllowed,
            (bool) shiftTurnKickPose,
            (const Rangea&) directionPrecision),
       vars((unsigned)(theMotionInfo.lastKickTimestamp) lastKickTimestamp)) /**< The value of the kick timestamp in the previous frame. */
{
  // This code should is executed for any state
  theMotionRequest.motion = MotionRequest::walkToBallAndKick;
  theMotionRequest.walkSpeed = speed;
  theMotionRequest.obstacleAvoidance = obstacleAvoidance;
  theMotionRequest.targetDirection = targetDirection;
  theMotionRequest.directionPrecision = directionPrecision;
  theMotionRequest.kickType = kickType;
  theMotionRequest.kickLength = kickLength;
  theMotionRequest.alignPrecisely = alignPrecisely;
  theMotionRequest.preStepType = preStepType;
  theMotionRequest.turnKickAllowed = turnKickAllowed;
  theMotionRequest.shiftTurnKickPose = shiftTurnKickPose;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.lastKickTimestamp != lastKickTimestamp)
        goto done;
    }
  }

  target_state(done)
  {
    transition
    {
      goto execute;
    }
    action
    {
      lastKickTimestamp = theMotionInfo.lastKickTimestamp;
    }
  }
}

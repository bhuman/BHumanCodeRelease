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
            (float) kickLength))
{
  theMotionRequest.motion = MotionRequest::dribble;
  theMotionRequest.walkSpeed = speed;
  theMotionRequest.obstacleAvoidance = obstacleAvoidance;
  theMotionRequest.targetDirection = targetDirection;
  theMotionRequest.alignPrecisely = alignPrecisely;
  theMotionRequest.kickType = KickInfo::rlKickDynamic; // Dummy kick type to ensure soft kick
  theMotionRequest.kickLength = kickLength;
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

/**
 * @file PenaltyStrikerGoToBallAndKick.cpp
 *
 * This file implements the PenaltyStrikerGoToBallAndKick skill.
 *
 * @author Arne Hasselbring (based on old behavior by Andreas Stolpmann)
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PenaltyStrikerGoToBallAndKick,
       args((const Pose2f&) kickPose,
            (KickInfo::KickType) kickType,
            (float) walkSpeed),
       defs((float)(200.f) prepareKickDistance, /**< A wait phase is inserted when this distance to the kick pose is reached. */
            (int)(4000) prepareKickDuration, /**< The duration to wait before the final approach. */
            (int)(10000) minTimeForKick)) /**< The minimum time needed for a regular penalty kick. If less time remains, skip preparation and directly kick the ball. */
{
  const int timeUntilPenaltyShootoutEnds = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);

  initial_state(goToBall)
  {
    transition
    {
      if(timeUntilPenaltyShootoutEnds <= minTimeForKick)
        goto goToBallAndKick;

      if(kickPose.translation.squaredNorm() < sqr(prepareKickDistance))
        goto prepareKick;
    }
    action
    {
      LookAtBall();
      WalkToPoint({.target = kickPose,
                   .speed = {walkSpeed, walkSpeed, walkSpeed},
                   .rough = true,
                   .disableObstacleAvoidance = true,
                   .disableAligning = true,
                   .disableStanding = true});
    }
  }

  state(prepareKick)
  {
    transition
    {
      if(state_time > prepareKickDuration || timeUntilPenaltyShootoutEnds <= minTimeForKick)
        goto goToBallAndKick;
    }
    action
    {
      LookLeftAndRight({.startLeft = true,
                        .maxPan = 30_deg,
                        .tilt = 5.7_deg,
                        .speed = 20_deg});
      Stand();
    }
  }

  state(goToBallAndKick)
  {
    transition
    {
      if(action_done)
        goto done;
    }
    action
    {
      LookAtBall();
      WalkToBallAndKick({.targetDirection = Angle::normalize(kickPose.rotation - theKickInfo[kickType].rotationOffset),
                         .kickType = kickType,
                         .alignPrecisely = theKickInfo[kickType].motion == MotionPhase::walk ? KickPrecision::notPrecise : KickPrecision::precise,
                         .speed = {walkSpeed, walkSpeed, walkSpeed}});
    }
  }

  target_state(done)
  {
    action
    {
      LookAtBall();
      WalkToBallAndKick({.targetDirection = Angle::normalize(kickPose.rotation - theKickInfo[kickType].rotationOffset),
                         .kickType = kickType,
                         .alignPrecisely = theKickInfo[kickType].motion == MotionPhase::walk ? KickPrecision::notPrecise : KickPrecision::precise,
                         .speed = {walkSpeed, walkSpeed, walkSpeed}});
    }
  }
}

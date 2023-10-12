/**
 * @file PenaltyStrikerGoToBallAndKick.cpp
 *
 * This file implements an implementation of the PenaltyStrikerGoToBallAndKick skill.
 *
 * @author Arne Hasselbring (based on old behavior by Andreas Stolpmann)
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/KickInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(PenaltyStrikerGoToBallAndKickImpl,
{,
  IMPLEMENTS(PenaltyStrikerGoToBallAndKick),
  CALLS(LookAtBall),
  CALLS(LookLeftAndRight),
  CALLS(Stand),
  CALLS(WalkToBallAndKick),
  CALLS(WalkToPoint),
  REQUIRES(KickInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(200.f) prepareKickDistance, /**< A wait phase is inserted when this distance to the kick pose is reached. */
    (int)(4000) prepareKickDuration, /**< The duration to wait before the final approach. */
  }),
});

class PenaltyStrikerGoToBallAndKickImpl : public PenaltyStrikerGoToBallAndKickImplBase
{
  option(PenaltyStrikerGoToBallAndKick)
  {
    initial_state(goToBall)
    {
      transition
      {
        if(p.kickPose.translation.squaredNorm() < sqr(prepareKickDistance))
          goto prepareKick;
      }
      action
      {
        theLookAtBallSkill();
        theWalkToPointSkill({.target = p.kickPose,
                             .speed = {p.walkSpeed, p.walkSpeed, p.walkSpeed},
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
        if(state_time > prepareKickDuration)
          goto goToBallAndKick;
      }
      action
      {
        theLookLeftAndRightSkill({.startLeft = true,
                                  .maxPan = 30_deg,
                                  .tilt = 5.7_deg,
                                  .speed = 20_deg});
        theStandSkill();
      }
    }

    state(goToBallAndKick)
    {
      transition
      {
        if(theWalkToBallAndKickSkill.isDone())
          goto done;
      }
      action
      {
        theLookAtBallSkill();
        theWalkToBallAndKickSkill({.targetDirection = Angle::normalize(p.kickPose.rotation - theKickInfo[p.kickType].rotationOffset),
                                   .kickType = p.kickType,
                                   .alignPrecisely = theKickInfo[p.kickType].motion == MotionPhase::walk ? KickPrecision::notPrecise : KickPrecision::precise,
                                   .speed = {p.walkSpeed, p.walkSpeed, p.walkSpeed}});
      }
    }

    target_state(done)
    {
      action
      {
        theLookAtBallSkill();
        theWalkToBallAndKickSkill({.targetDirection = Angle::normalize(p.kickPose.rotation - theKickInfo[p.kickType].rotationOffset),
                                   .kickType = p.kickType,
                                   .alignPrecisely = theKickInfo[p.kickType].motion == MotionPhase::walk ? KickPrecision::notPrecise : KickPrecision::precise,
                                   .speed = {p.walkSpeed, p.walkSpeed, p.walkSpeed}});
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(PenaltyStrikerGoToBallAndKickImpl);

/**
 * @file ReceivePass.cpp
 *
 * This file defines an implementation of the ReceivePass skill, that makes the player walk to the ball's target position that the kicking teammate has communicated.
 *
 * @author Jo Lienhoop
 */

#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Math/BHMath.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/KickSelection.h"

SKILL_IMPLEMENTATION(ReceivePassImpl,
{,
  IMPLEMENTS(ReceivePass),
  CALLS(LookActive),
  CALLS(LookAtBallAndTarget),
  CALLS(Stand),
  CALLS(WalkToPoint),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
});

class ReceivePassImpl : public ReceivePassImplBase
{
  void execute(const ReceivePass& p) override
  {
    // Find the teammate that's passing the ball to this player
    const Teammate* teammate = nullptr;
    for(const Teammate& t : theTeamData.teammates)
    {
      if(t.number == p.playerNumber)
      {
        teammate = &t;
        break;
      }
    }
    if(!teammate)
    {
      p.setState("invalidPassingPlayer");
      return;
    }

    const Vector2f passBase = theFieldBall.recentBallPositionOnField();
    // Set the target position to where the passing teammate communicated kicking the ball to
    const Vector2f passTarget = teammate->theRobotPose * teammate->theBehaviorStatus.shootingTo.value_or(Vector2f::Zero());
    const Angle targetAngle = Angle::normalize(KickSelection::calculateTargetRotation(passBase, passTarget, opponentGoal) - theRobotPose.rotation);

    // Walk to the target position and look towards the ball to receive it when it arrives
    const Pose2f passTargetRelative(targetAngle, theRobotPose.inverse() * passTarget);
    theWalkToPointSkill({.target = passTargetRelative,
                         .targetOfInterest = theFieldBall.recentBallPositionRelative(),
                         .forceSideWalking = false});
    theLookActiveSkill({.withBall = true,
                        .onlyOwnBall = false});

    COMPLEX_DRAWING("skill:ReceivePass:target")
    {
      CROSS("skill:ReceivePass:target", passTarget.x(), passTarget.y(), 100, 20, Drawings::solidPen, ColorRGBA::blue);
      LINE("skill:ReceivePass:target", passBase.x(), passBase.y(), passTarget.x(), passTarget.y(), 20, Drawings::solidPen, ColorRGBA::blue);
    }
  }

  void preProcess() override {}
  void preProcess(const ReceivePass&) override
  {
    DECLARE_DEBUG_DRAWING("skill:ReceivePass:target", "drawingOnField");
  }

  const Vector2f opponentGoal = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
};

MAKE_SKILL_IMPLEMENTATION(ReceivePassImpl);

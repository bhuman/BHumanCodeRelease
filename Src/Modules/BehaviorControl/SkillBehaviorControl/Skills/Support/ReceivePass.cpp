/**
 * @file ReceivePass.cpp
 *
 * This file defines an implementation of the ReceivePass skill, that makes the player walk to the ball's target position that the kicking teammate has communicated.
 *
 * @author Jo Lienhoop
 */

#include "SkillBehaviorControl.h"
#include "Debugging/DebugDrawings.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include <algorithm>

option((SkillBehaviorControl) ReceivePass, args((int) playerNumber),
       defs((Angle)(30_deg) maxAngle)) /**< maximum angle to the ball that it can still be received in the front */
{
  const Vector2f opponentGoal = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);
  const auto teammate = std::find_if(theTeamData.teammates.begin(), theTeamData.teammates.end(),
                                     [&](const Teammate& t) {return t.number == playerNumber;});

  common_transition
  {
    if(teammate == theTeamData.teammates.end())
      goto invalidPassingPlayer;
    else
      goto receive;
  }

  initial_state(receive)
  {
    action
    {
      const Vector2f passBase = theFieldBall.recentBallPositionOnField();
      // Set the target position to where the passing teammate communicated kicking the ball to
      const Vector2f passTarget = teammate->theRobotPose * teammate->theBehaviorStatus.shootingTo.value_or(Vector2f::Zero());
      const Angle targetAngle = Angle::normalize(KickSelection::calculateTargetRotation(passBase, passTarget, opponentGoal, maxAngle) - theRobotPose.rotation);

      // Walk to the target position and look towards the ball to receive it when it arrives
      const Pose2f passTargetRelative(targetAngle, theRobotPose.inverse() * passTarget);
      WalkToPoint({.target = passTargetRelative,
                   .reduceWalkingSpeed = ReduceWalkSpeedType::normal,
                   .targetOfInterest = theFieldBall.recentBallPositionRelative(),
                   .forceSideWalking = false});
      LookActive({.withBall = true,
                  .onlyOwnBall = false,
                  .slowdownFactor = 0.5f});

      COMPLEX_DRAWING("option:ReceivePass:target")
      {
        CROSS("option:ReceivePass:target", passTarget.x(), passTarget.y(), 100, 20, Drawings::solidPen, ColorRGBA::blue);
        LINE("option:ReceivePass:target", passBase.x(), passBase.y(), passTarget.x(), passTarget.y(), 20, Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }

  aborted_state(invalidPassingPlayer) {}
}

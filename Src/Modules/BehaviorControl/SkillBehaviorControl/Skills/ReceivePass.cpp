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
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/BehaviorControl/KickSelection.h"

SKILL_IMPLEMENTATION(ReceivePassImpl,
{,
  IMPLEMENTS(ReceivePass),
  CALLS(LookActive),
  CALLS(LookAtBallAndTarget),
  CALLS(Stand),
  CALLS(WalkToPoint),
  REQUIRES(BallModel),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(KickInfo),
  REQUIRES(MotionInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(WalkingEngineOutput),
  LOADS_PARAMETERS(
  {,
    (bool) forceStanding, /**< If true, hysterese is used to determine if the robot should start/stop standing. */
    (float) translationStopThreshold, /**< Threshold for the translation to stop walking. */
    (float) translationStartThreshold, /**< Threshold for the translation to start walking. */
    (Angle) rotationStopThreshold, /**< Threshold for the rotation to stop walking. */
    (Angle) rotationStartThreshold, /**< Threshold for the rotation to start walking. */
  }),
});

class ReceivePassImpl : public ReceivePassImplBase
{
  bool isStanding = false;

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
    const Vector2f passTarget = teammate->theRobotPose * teammate->theBehaviorStatus.shootingTo;
    const Angle targetAngle = Angle::normalize((passBase - passTarget).angle() - theRobotPose.rotation);

    // Walk to the target position and look towards the ball to receive it when it arrives
    const Pose2f passTargetRelative(targetAngle, theRobotPose.inversePose * passTarget);
    isStanding |= theMotionInfo.isMotion(MotionPhase::stand);
    if(!forceStanding ||
       (!isStanding && !stopWalking(passTargetRelative)) || // When currently walking, check if we should stop walking
       (isStanding && startWalking(passTargetRelative))) // When currently standing, check if we should start walking
    {
      theWalkToPointSkill({.target = {targetAngle, theRobotPose.inversePose * passTarget},
                           .targetOfInterest = theFieldBall.recentBallPositionRelative(),
                           .forceSideWalking = true});
      isStanding = false;
    }
    else
    {
      theStandSkill({.high = false});
      isStanding = true;
    }
    theLookActiveSkill({.withBall = true,
                        .onlyOwnBall = true});
    // theLookAtBallAndTargetSkill({.walkingDirection = theRobotPose.inversePose * passTarget, .ballPositionAngle = targetAngle});

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

  bool startWalking(const Pose2f& passTargetRelative)
  {
    return std::abs(passTargetRelative.rotation) > rotationStartThreshold ||
           std::abs(passTargetRelative.translation.x()) > translationStartThreshold ||
           std::abs(passTargetRelative.translation.y()) > translationStartThreshold;
  }

  bool stopWalking(const Pose2f& passTargetRelative)
  {
    return std::abs(passTargetRelative.rotation) < rotationStopThreshold &&
           std::abs(passTargetRelative.translation.x()) < translationStopThreshold &&
           std::abs(passTargetRelative.translation.y()) < translationStopThreshold;
  }
};

MAKE_SKILL_IMPLEMENTATION(ReceivePassImpl);

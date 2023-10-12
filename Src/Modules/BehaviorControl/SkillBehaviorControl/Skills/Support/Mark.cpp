/**
 * @file Mark.cpp
 *
 * This file implements an implementation of the Mark skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Debugging/DebugDrawings.h"

SKILL_IMPLEMENTATION(MarkImpl,
{,
  IMPLEMENTS(Mark),
  CALLS(LookActive),
  CALLS(WalkToPoint),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(750.f) distToMarkedRobot, /**< The minimum distance a robot must have to the marked robot. */
  }),
});

class MarkImpl : public MarkImplBase
{
  void execute(const Mark& p) override
  {
    const Pose2f markPose = calcMarkPose(p.target);
    DRAW_ROBOT_POSE("skill:MarkImpl:markPose", markPose, ColorRGBA::red);
    theLookActiveSkill({.withBall = true});
    theWalkToPointSkill({.target = markPose});
  }

  Pose2f calcMarkPose(const Vector2f& target) const
  {
    const Vector2f ballPositionField(theFieldBall.recentBallPositionOnField());
    const Vector2f gloObstacleToBlock = theRobotPose * target;

    Vector2f positionOnField = gloObstacleToBlock + (Vector2f(theFieldDimensions.xPosOwnGroundLine, 0.f) - gloObstacleToBlock).normalized(distToMarkedRobot);
    return theRobotPose.inverse() * Pose2f(((ballPositionField - positionOnField).normalized() + (gloObstacleToBlock - positionOnField).normalized()).angle(), positionOnField);
  }

  void preProcess(const Mark&) override
  {
    DECLARE_DEBUG_DRAWING("skill:MarkImpl:markPose", "drawingOnField");
  }

  void preProcess() override {}
};

MAKE_SKILL_IMPLEMENTATION(MarkImpl);

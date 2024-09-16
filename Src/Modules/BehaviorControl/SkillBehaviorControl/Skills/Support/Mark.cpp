/**
 * @file Mark.cpp
 *
 * This file implements the Mark skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"
#include "Debugging/DebugDrawings.h"

option((SkillBehaviorControl) Mark,
       args((const Vector2f&) target),
       defs((float)(750.f) distToMarkedRobot)) /**< The minimum distance a robot must have to the marked robot. */
{
  initial_state(execute)
  {
    action
    {
      const Vector2f ballPositionField(theFieldBall.recentBallPositionOnField());
      const Vector2f gloObstacleToBlock = theRobotPose * target;
      const Vector2f positionOnField = gloObstacleToBlock + (Vector2f(theFieldDimensions.xPosOwnGoalLine, 0.f) - gloObstacleToBlock).normalized(distToMarkedRobot);
      const Pose2f markPose = theRobotPose.inverse() * Pose2f(((ballPositionField - positionOnField).normalized() + (gloObstacleToBlock - positionOnField).normalized()).angle(), positionOnField);
      DRAW_ROBOT_POSE("option:Mark:markPose", markPose, ColorRGBA::red);
      LookActive({.withBall = true});
      WalkToPoint({.target = markPose});
    }
  }
}

/**
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */
#include "GoaliePoseProvider.h"

#include "Platform/SystemCall.h"
#include "Framework/Settings.h"
#include "Math/Geometry.h"
#include "Math/Pose2f.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Debugging/DebugDrawings.h"

MAKE_MODULE(GoaliePoseProvider);

void GoaliePoseProvider::update(GoaliePose& goaliePose)
{
  DECLARE_DEBUG_DRAWING("module:GoaliePoseProvider:goaliePose", "drawingOnField");

  const float minXValue = theFieldDimensions.xPosOwnGroundLine + distanceToGroundLineForMinimumXValue;
  goaliePose.goaliePoseField = calcGlobalPoseOnBobLineAndCut();
  ASSERT(goaliePose.goaliePoseField.isFinite());
  if(goaliePose.goaliePoseField.translation.x() < minXValue)
    goaliePose.goaliePoseField.translation.x() = minXValue;
  ASSERT(goaliePose.goaliePoseField.isFinite());

  goaliePose.goaliePoseRel = theRobotPose.inverse() * goaliePose.goaliePoseField;
  ASSERT(goaliePose.goaliePoseRel.isFinite());

  {
    const Vector2f leftPost(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal);
    const Vector2f rightPost(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal);
    const Vector2f leftRel = theRobotPose.inverse() * leftPost;
    const Vector2f rightRel = theRobotPose.inverse() * rightPost;
    const float angleToLeft = leftRel.angle();
    const float angleToRight = rightRel.angle();
    goaliePose.isNearLeftPost =
      leftRel.norm() < jumpDistanceThreshold &&
      angleToLeft > jumpAngleThresholdMin &&
      angleToLeft < jumpAngleThresholdMax;
    goaliePose.isNearRightPost =
      rightRel.norm() < jumpDistanceThreshold &&
      angleToRight < -jumpAngleThresholdMin &&
      angleToRight > -jumpAngleThresholdMax;
  }

  CROSS("module:GoaliePoseProvider:goaliePose",
        goaliePose.goaliePoseField.translation.x(),
        goaliePose.goaliePoseField.translation.y(), 40, 20, Drawings::solidPen, ColorRGBA::red);
}

Pose2f GoaliePoseProvider::calcGlobalBobPose() const
{
  const Vector2f gloPos = theFieldBall.recentBallPositionOnField(timeOutBallLastSeen, timeOutBallDisappear);

  Vector2f gloBall = gloPos;
  gloBall.x() = std::max(gloPos.x(), theFieldDimensions.xPosOwnGroundLine + distanceToGroundLineForMinimumXValue);
  gloBall.y() = std::abs(gloBall.y()) < theFieldDimensions.yPosLeftFieldBorder ? gloBall.y() : sgn(gloBall.y()) * theFieldDimensions.yPosLeftFieldBorder;

  ASSERT(gloBall.allFinite());

  const Vector2f leftPost(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal);
  const Vector2f rightPost(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal);

  const Vector2f ballToLeftPost = leftPost - gloBall;
  const Vector2f ballToRightPost = rightPost - gloBall;

  Angle openingAngle = ballToLeftPost.angleTo(ballToRightPost);
  ASSERT(openingAngle > 0.f);

  const float dist = std::sqrt(sqr(2.f * theBehaviorParameters.genuflectRadius) / (2.f * (1.f - std::cos(openingAngle))));

  Pose2f bobPose;
  bobPose.translation = gloBall + Vector2f(dist, 0.f).rotate(ballToLeftPost.angle() + openingAngle / 2.f);
  bobPose.rotation = (gloBall - bobPose.translation).angle();

  ASSERT(bobPose.isFinite());
  return bobPose;
}

Pose2f GoaliePoseProvider::calcGlobalPoseOnBobLine() const
{
  const Pose2f bobPose(calcGlobalBobPose());
  const Geometry::Line bobLine(bobPose);
  const Vector2f goalPost(theFieldDimensions.xPosOwnGroundLine, bobPose.rotation > 0 ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);
  const Angle rotation(bobPose.rotation > 0 ? positionAngle : -positionAngle);
  const Geometry::Line postLine(Pose2f(Angle(bobPose.rotation + rotation).normalize(), goalPost));
  Vector2f intersection;
  VERIFY(Geometry::getIntersectionOfLines(bobLine, postLine, intersection));
  return Pose2f(bobPose.rotation, intersection);
}

Pose2f GoaliePoseProvider::calcGlobalPoseOnBobLineAndCut() const
{
  const Pose2f pose(calcGlobalPoseOnBobLine());
  ASSERT(pose.isFinite());
  const float maxValueX = theFieldDimensions.xPosOwnGoalArea + goalieLine;
  Vector2f intersection(pose.translation);
  if(pose.translation.x() > maxValueX)
    VERIFY(Geometry::getIntersectionOfLines(Geometry::Line(pose), Geometry::Line(Pose2f(90_deg, maxValueX, 0.f)), intersection));
  return Pose2f(pose.rotation, intersection);
}

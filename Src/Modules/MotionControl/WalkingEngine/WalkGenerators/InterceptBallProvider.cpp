/**
 * @file InterceptBallProvider.cpp
 *
 * This file implements a module that intercepts the ball
 *
 * @author Philip Reichenberg
 */

#include "InterceptBallProvider.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/Transformation.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Libs/Math/Approx.h"

MAKE_MODULE(InterceptBallProvider);

using namespace Motion::Transformation;

void InterceptBallProvider::update(InterceptBallGenerator& theInterceptBallGenerator)
{
  if(!calculatedRectangles)
  {
    backRightIntercepting = Vector2f(0.f, -theRobotDimensions.yHipOffset);
    frontLeftIntercepting = Vector2f(bestHitPoint, theRobotDimensions.yHipOffset);
    calculatedRectangles = true;
  }

  theInterceptBallGenerator.intercept = [this](const MotionRequest& motionRequest, const std::vector<Vector2f>& translationPolygon, const bool isLeftPhase, const std::optional<Vector2f>& oldStep)
  {
    return intercept(motionRequest, 0, isLeftPhase, translationPolygon, oldStep);
  };

  theInterceptBallGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    // intercept ball
    return theWalkGenerator.createPhase(intercept(motionRequest, &lastPhase, theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position), std::vector<Vector2f>(), std::optional<Vector2f>()), lastPhase, 0.f);
  };
}

Pose2f InterceptBallProvider::intercept(const MotionRequest& motionRequest, const MotionPhase* lastPhase, const bool isLeftPhase, const std::vector<Vector2f>& translationPolygon, const std::optional<Vector2f>& oldStep)
{
  // Determine which velocity shall be used
  Vector2f useBallVelocity = motionRequest.ballEstimate.velocity;
  if(motionRequest.ballEstimate.velocity.squaredNorm() < 1.f)
  {
    Vector2f ballDirection = theFieldInterceptBall.interceptedEndPositionRelative - motionRequest.ballEstimate.position;
    if(ballDirection.squaredNorm() == 0.f)
    {
      ballDirection = theFieldInterceptBall.intersectionPositionWithOwnYAxis - motionRequest.ballEstimate.position;
      ASSERT(ballDirection.squaredNorm() != 0.f); // Once x Intersection is actually used, this needs to be adjusted
    }

    // Something is broken, walk in place until behavior functions normally again
    if(ballDirection.squaredNorm() == 0.f)
    {
      return Pose2f(0.f, 0.001f, 0.f);
    }

    useBallVelocity = ballDirection.normalized(BallPhysics::velocityForDistance(ballDirection.norm(), theBallSpecification.friction));
  }

  // Get transformation matrix from robotpose-relative coordinates to start-of-next-walkstep-relative coordinates
  const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theRobotDimensions, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);

  // Determine orth point
  const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, useBallVelocity.rotated(scsCognition.rotation));
  Vector2f orthPoint = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, Vector2f(bestHitPoint, 0.f));
  bool interpolateKickPose = false;
  bool noKickPose = false;
  Vector2f p1, p2, p3;
  const bool isRightIntercept = ballLine.base.y() < 0.f == Legs::right;
  if(Approx::isEqual((orthPoint - ballLine.base).angle(), ballLine.direction.angle(), static_cast<float>(1_deg)))
  {
    if(Geometry::getIntersectionPointsOfLineAndRectangle(backRightIntercepting, frontLeftIntercepting, ballLine, p1, p2) &&
       p1.x() != p2.x())
    {
      const float interpolationFactor = mapToRange(0.f, p1.x(), p2.x(), 0.f, 1.f);
      p3 = p2 * interpolationFactor + p1 * (1.f - interpolationFactor);
      interpolateKickPose = true;
      noKickPose = true;
    }
    else
    {
      const float hipOffset = isRightIntercept ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset;
      // We want to optimize for as less walk step size as possible
      interpolateKickPose = Geometry::getIntersectionOfLines(ballLine, Geometry::Line(Vector2f(0.f, hipOffset), Vector2f(1.f, 0.f)), p3);
    }
  }
  else
    orthPoint = ballLine.base; // Ball rolled already passed us. There is no correct orth point
  // Calculate kickPose
  Pose2f kickPose = orthPoint;
  kickPose.translate(Vector2f(-bestHitPoint, 0.f));
  if(interpolateKickPose)
  {
    Pose2f kickPoseOther = p3;
    if(!noKickPose)
      kickPoseOther.translate(Vector2f(-bestHitPoint, 0.f));

    const Vector2f closestPoint = Geometry::getOrthogonalProjectionOfPointOnLine(Geometry::Line(kickPose.translation, kickPoseOther.translation - kickPose.translation), Vector2f::Zero());
    const Vector2f& minPose = kickPose.translation.x() < kickPoseOther.translation.x() ? kickPose.translation : kickPoseOther.translation;
    const Vector2f& maxPose = kickPose.translation.x() >= kickPoseOther.translation.x() ? kickPose.translation : kickPoseOther.translation;
    const Vector2f& minOrth = kickPose.translation.x() < kickPoseOther.translation.x() ? orthPoint : p3;
    const Vector2f& maxOrth = kickPose.translation.x() >= kickPoseOther.translation.x() ? orthPoint : p3;
    if(minPose.x() != maxPose.x())
    {
      const float interpolationFactor = mapToRange(closestPoint.x(), minPose.x(), maxPose.x(), 0.f, 1.f);
      kickPose.translation = (1.f - interpolationFactor) * minPose + interpolationFactor * maxPose;
      orthPoint = (1.f - interpolationFactor) * minOrth + interpolationFactor * maxOrth;
    }
  }
  // Get time, ball roll angle and size of shifted contact width
  float timeForDistance = BallPhysics::timeForDistance(useBallVelocity, (orthPoint - ballLine.base).norm(), theBallSpecification.friction);
  if(std::numeric_limits<float>::max() == timeForDistance)
    timeForDistance = BallPhysics::computeTimeUntilBallStops(useBallVelocity, theBallSpecification.friction);

  // Ball is close. Reduce side step size so the ball is just touched with the tip of one of the feet.
  // This prevent the ball to roll through between both legs
  // Also apply this clip before clipPose(). Otherwise the step size is reduced by more than it is intended
  if(timeForDistance < 1.f)
  {
    const Angle correctionAngle = Angle::normalize(ballLine.direction.angle() + 180_deg); // 360deg is one circle, half is 180deg. The direction is just mirrored
    const float yShiftClipped = maxYShift.limit(-Vector2f(theRobotDimensions.footLength, 0.f).rotate(correctionAngle).y());
    const float sideScaling = mapToRange(timeForDistance, 0.5f, 1.f, 1.f, 0.f);
    const Rangef yRange(sideScaling * (-theRobotDimensions.yHipOffset + footTipYShift) + std::max(yShiftClipped, 0.f), sideScaling * (theRobotDimensions.yHipOffset - footTipYShift) + std::min(yShiftClipped, 0.f));
    const float correctionValue = yRange.limit(kickPose.translation.y());
    if(correctionValue * kickPose.translation.y() > 0.f) // ball is rollin
      kickPose.translation.y() = kickPose.translation.y() - yRange.limit(kickPose.translation.y());
  }
  // previous reduction is intended to be applied
  if(timeForDistance < preventSideKickTime)
  {
    // Allow feet moving together, but not outwards
    Rangef clipSideRange(std::min(scsCognition.translation.y(), 0.f), std::max(scsCognition.translation.y(), 0.f));
    if(oldStep) // we are updating the current step. Allow for old previous value
    {
      clipSideRange.min = std::min(clipSideRange.min, oldStep.value().y());
      clipSideRange.max = std::max(clipSideRange.max, oldStep.value().y());
    }
    if((isLeftPhase && kickPose.translation.y() > 0.f) || (!isLeftPhase && kickPose.translation.y() < 0.f))
      kickPose.translation.y() = clipSideRange.limit(kickPose.translation.y());
  }
  if(lastPhase)
    clipPose(kickPose, isLeftPhase, *lastPhase, true);
  else
    clipPose(kickPose, isLeftPhase, translationPolygon);

  // Ball is soon intercepted. A bit more optimized handling
  if(timeForDistance < 1.f)
  {
    // We are close to the target. Start backwalking to not kick the ball away
    if(orthPoint.x() - bestHitPoint < 50.f)  // TODO not sure if this even triggers often enough. So no parameter for the 50!
      kickPose.translation.x() = maxXStep.min;
    else
      // Walk to second best pose
      kickPose.translation.x() = maxXStep.limit(kickPose.translation.x());
  }
  // else: We still got time to stand optimal

  // Ball is rolling at a steep angle -> rotate to allow for better sidewalking
  if(std::abs(ballLine.direction.angle()) < maxBallRollAngle)
    kickPose.rotation = theWalkGenerator.getRotationRange(isLeftPhase, Pose2f(1.f, 1.f, 1.f)).clamped(ballLine.direction.angle() < 0.f ? maxRotationStep : -maxRotationStep);

  // Clip rotation
  if(lastPhase)
    kickPose.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, Pose2f(1.f, 1.f, 1.f), kickPose.translation, false, *lastPhase, true, true).limit(kickPose.rotation);
  else
    kickPose.rotation = theWalkGenerator.getStepRotationRangeOther(isLeftPhase, Pose2f(1.f, 1.f, 1.f), kickPose.translation, false, translationPolygon, true, true).limit(kickPose.rotation);

  // wrong foot -> can't execute side step
  if(isLeftPhase == (kickPose.translation.y() < 0.f))
    kickPose.translation.y() = 0.f;

  ASSERT(!std::isnan(kickPose.translation.x()));
  ASSERT(!std::isnan(kickPose.translation.y()));
  ASSERT(!std::isnan(kickPose.rotation));
  return kickPose;
}

void InterceptBallProvider::clipPose(Pose2f& pose, bool isLeftPhase, const std::vector<Vector2f>& translationPolygon)
{
  const Vector2f& firstEdge = translationPolygon[0];

  if(std::abs(pose.translation.y()) >= std::abs(firstEdge.y()))
  {
    const float sign = static_cast<float>(sgnPos(pose.translation.y()));
    pose.translation.y() = firstEdge.y() * sign;
    if(pose.translation.x() >= 0.f)
      pose.translation.x() = std::min(firstEdge.x(), pose.translation.x());
    else
    {
      ASSERT(translationPolygon.size() >= 4);
      const Vector2f& lastEdge = translationPolygon[translationPolygon.size() - 1];
      pose.translation.x() = std::max(lastEdge.x(), pose.translation.x());
    }
  }
  else if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), pose.translation))
  {
    Vector2f intersectionPoint;
    VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, pose.translation.y()), Vector2f(pose.translation.x(), 0.f)), intersectionPoint, false));
    pose.translation = intersectionPoint;
  }

  if(isLeftPhase == (pose.translation.y() < 0.f))
    pose.translation.y() = 0.f;
}

void InterceptBallProvider::clipPose(Pose2f& pose, bool isLeftPhase, const MotionPhase& lastPhase, const bool isIntercepting)
{
  std::vector<Vector2f> translationPolygon;
  std::vector<Vector2f> translationPolygonNoCenter;
  // 0 Rotation, because we want full translation. Just ensure the rotation is later clipped
  theWalkGenerator.getTranslationPolygon(isLeftPhase, 0, lastPhase, Pose2f(1.f, 1.f, 1.f), translationPolygon, translationPolygonNoCenter, false, isIntercepting);

  clipPose(pose, isLeftPhase, translationPolygon);
}

/**
 * @file InterceptBallProvider.cpp
 *
 * This file implements a module that intercepts the ball
 *
 * @author Florian Scholz
 */

#include "InterceptBallProvider.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/Transformation.h"

MAKE_MODULE(InterceptBallProvider);

using namespace Motion::Transformation;

void InterceptBallProvider::update(InterceptBallGenerator& theInterceptBallGenerator)
{
  DEBUG_DRAWING("motion:RobotPose", "drawingOnField") // Set the origin to the robot's current position
  {
    ORIGIN("motion:RobotPose", theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation);
  }

  DECLARE_DEBUG_DRAWING("interceptBallProvider:hitPos", "drawingOnField");
  DECLARE_DEBUG_DRAWING("interceptBallProvider:line", "drawingOnField");
  DECLARE_DEBUG_DRAWING("interceptBallProvider:rect", "drawingOnField");

  leftFootBackRight = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius, theRobotDimensions.yHipOffset - footRectYOffset);
  leftFootFrontLeft = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectLengthOffset, theRobotDimensions.yHipOffset + footRectYOffset);

  rightFootBackRight = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius, -theRobotDimensions.yHipOffset - footRectYOffset);
  rightFootFrontLeft = Vector2f(theRobotDimensions.soleToFrontEdgeLength + theBallSpecification.radius + footRectLengthOffset, -theRobotDimensions.yHipOffset + footRectYOffset);

  leftFootRect = Geometry::Rect(leftFootBackRight, leftFootFrontLeft);
  rightFootRect = Geometry::Rect(rightFootBackRight, rightFootFrontLeft);

  theInterceptBallGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    Vector2f ballVelocity = motionRequest.ballEstimate.velocity;

    if(motionRequest.ballEstimate.velocity.squaredNorm() < 1.f)
    {
      const Vector2f ballDirection = theFieldBall.interceptedEndPositionRelative - motionRequest.ballEstimate.position;
      ASSERT(ballDirection.squaredNorm() != 0.f);

      if(ballDirection.squaredNorm() == 0.f)
        return theWalkGenerator.createPhase(Pose2f(0.f, 0.001f, 0.f), lastPhase, 0.f);

      ballVelocity = ballDirection.normalized(BallPhysics::velocityForDistance(ballDirection.norm(), theBallSpecification.friction));
    }

    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position);
    const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theRobotDimensions, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);

    const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, ballVelocity.rotated(scsCognition.rotation));

    const KickInfo::KickType kicktype = motionRequest.kickType;

    Vector2f orthPoint = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, -(theKickInfo[kicktype].ballOffset.rotated(scsCognition.rotation + motionRequest.targetDirection + theKickInfo[kicktype].rotationOffset)));

    Pose2f kickPose = Pose2f(scsCognition.rotation + motionRequest.targetDirection, orthPoint);

    kickPose.rotate(theKickInfo[kicktype].rotationOffset);
    kickPose.rotation = Angle::normalize(kickPose.rotation);
    kickPose.translate(theKickInfo[kicktype].ballOffset);

    if(kickPose.translation.x() < 0)
    {
      const float b = std::abs(orthPoint.x() + theKickInfo[kicktype].ballOffset.x());

      Angle alpha = ballLine.direction.angle();

      if(std::abs(alpha) > 90_deg)
        alpha = Angle::normalize(alpha + 180_deg);

      const float a = std::tan(alpha) * b;

      orthPoint += Vector2f(b, a);

      kickPose = Pose2f(scsCognition.rotation + motionRequest.targetDirection, orthPoint);

      kickPose.rotate(theKickInfo[kicktype].rotationOffset);
      kickPose.rotation = Angle::normalize(kickPose.rotation);
      kickPose.translate(theKickInfo[kicktype].ballOffset);
    }

    clipPose(kickPose, isLeftPhase, lastPhase, motionRequest);

    kickPose.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, motionRequest.walkSpeed, kickPose.translation, false, lastPhase, true).limit(kickPose.rotation);

    if(!checkKickType(kicktype))
      return theWalkGenerator.createPhase(kickPose, lastPhase, 0.f);

    const Vector2f drawableOrth = scsCognition.inverse() * orthPoint;

    CROSS("interceptBallProvider:hitPos", drawableOrth.x(), drawableOrth.y(), 100, 20, Drawings::solidPen, ColorRGBA::magenta);

    LINE("interceptBallProvider:line", motionRequest.ballEstimate.position.x(), motionRequest.ballEstimate.position.y(), motionRequest.ballEstimate.position.x() + ballVelocity.x(), motionRequest.ballEstimate.position.y() + ballVelocity.y(), 20, Drawings::solidPen, ColorRGBA::red);

    WalkKickVariant walkKickVariant = WalkKickVariant(kicktype, theKickInfo[kicktype].walkKickType, theKickInfo[kicktype].kickLeg, KickPrecision::justHitTheBall, motionRequest.kickLength, motionRequest.targetDirection, false);

    Vector2f timeForDistanceVector = orthPoint;

    const Vector2f hitPoint(0.f, walkKickVariant.kickLeg == Legs::right ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Geometry::Line hitPointLine(hitPoint, Vector2f(1.f, 0.f));
    Vector2f hitPointIntersection;
    if(Geometry::getIntersectionOfLines(hitPointLine, ballLine, hitPointIntersection))
    {
      const Vector2f hitPointLower = rightFootRect.a;
      const Geometry::Line hitPointLowerLine(hitPointLower, Vector2f(0.f, 1.f));
      Vector2f hitPointLowerIntersection;
      if(Geometry::getIntersectionOfLines(hitPointLowerLine, ballLine, hitPointLowerIntersection))
      {
        const Vector2f hitPointUpper = rightFootRect.b;
        const Geometry::Line hitPointUpperLine(hitPointUpper, Vector2f(0.f, 1.f));
        Vector2f hitPointUpperIntersection;
        VERIFY(Geometry::getIntersectionOfLines(hitPointUpperLine, ballLine, hitPointUpperIntersection));

        const float hitPointUpperToHitPointDistance = (hitPointIntersection - hitPointUpperIntersection).norm();
        const float hitPointLowerToHitPointDistance = (hitPointIntersection - hitPointLowerIntersection).norm();
        const float orthPointToHitPointDistance = (hitPointIntersection - orthPoint).norm();

        if(hitPointLowerToHitPointDistance <= hitPointUpperToHitPointDistance && hitPointLowerToHitPointDistance <= orthPointToHitPointDistance)
        {
          timeForDistanceVector = hitPointLowerIntersection;
        }
        else if(hitPointUpperToHitPointDistance <= hitPointLowerToHitPointDistance && hitPointUpperToHitPointDistance <= orthPointToHitPointDistance)
        {
          timeForDistanceVector = hitPointUpperIntersection;
        }
        else
        {
          timeForDistanceVector = orthPoint;
        }
      }
      else
      {
        timeForDistanceVector = hitPointIntersection;
      }
    }

    const float timeForDistanceHitPoint = BallPhysics::timeForDistance(ballVelocity, (timeForDistanceVector - ballLine.base).norm(), theBallSpecification.friction);

    const float timeForDistanceOrthPoint = BallPhysics::timeForDistance(ballVelocity, (orthPoint - ballLine.base).norm(), theBallSpecification.friction);

    walkKickVariant.ballEstimationTime = timeForDistanceHitPoint;

    DelayKickParams delayParams;
    delayParams.delay = timeForDistanceHitPoint - timeOffset;
    delayParams.kickIndex = 1;
    walkKickVariant.delayParams = delayParams;

    // only used to modify the walkKicKVariant (mainly direction), return is ignored intentionally
    theWalkKickGenerator.canStart(walkKickVariant, lastPhase, motionRequest.directionPrecision, true, false);

    Geometry::Rect drawableRect = walkKickVariant.kickLeg == Legs::right ? rightFootRect : leftFootRect;
    drawableRect.a = scsCognition.inverse() * drawableRect.a;
    drawableRect.b = scsCognition.inverse() * drawableRect.b;

    RECTANGLE("interceptBallProvider:rect", drawableRect.a.x(), drawableRect.a.y(), drawableRect.b.x(), drawableRect.b.y(), 15, Drawings::solidPen, ColorRGBA::black);

    const bool orthPointIsInsideKickingRectangle = Geometry::isPointInsideRectangle(walkKickVariant.kickLeg == Legs::right ? rightFootRect : leftFootRect, orthPoint);

    // if we can kick (position and timing wise)
    if(orthPointIsInsideKickingRectangle && timeForDistanceHitPoint - timeOffset < 0.6f)
    {
      auto nextPhase = theWalkKickGenerator.createPhase(walkKickVariant, lastPhase, false);
      if(nextPhase)
        return nextPhase;
    }

    // if we cannot kick position wise but timing wise
    if(timeForDistanceOrthPoint - timeOffset < 0.6f)
      kickPose = orthPoint;

    const Pose2f lastPose = theWalkGenerator.getLastStepChange(lastPhase);
    // if we cannot make another step before we kick the ball (because of time to distance) AND the step we want to make would be a reverse of the last step (on the x-axis). Then we dont move sideways
    if(std::abs(lastPose.translation.y() + kickPose.translation.y()) < 10.f &&
       timeForDistanceOrthPoint - timeOffset < 1.f)
    {
      kickPose = Vector2f(orthPoint.x(), 0.f);
    }

    clipPose(kickPose, isLeftPhase, lastPhase, motionRequest);
    return theWalkGenerator.createPhase(kickPose, lastPhase, 0.f);
  };
}

void InterceptBallProvider::clipPose(Pose2f& pose, bool isLeftPhase, const MotionPhase& lastPhase, const MotionRequest& motionRequest)
{
  std::vector<Vector2f> translationPolygon;
  std::vector<Vector2f> translationPolygonNoCenter;
  theWalkGenerator.getTranslationPolygon(isLeftPhase, pose.rotation, lastPhase, motionRequest.walkSpeed, translationPolygon, translationPolygonNoCenter, false, false);

  const Vector2f& firstEdge = translationPolygon[0];

  if(std::abs(pose.translation.y()) >= std::abs(firstEdge.y()))
  {
    const float sign = static_cast<float>(sgnPos(pose.translation.y()));
    pose.translation.y() = firstEdge.y() * sign;
    pose.translation.x() = std::min(firstEdge.x(), pose.translation.x());
  }
  else if(!Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), pose.translation))
  {
    Vector2f intersectionPoint;
    VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, pose.translation.y()), pose.translation), intersectionPoint));
    pose.translation = intersectionPoint;
  }

  if(isLeftPhase == (pose.translation.y() < 0.f))
    pose.translation.y() = 0.f;
}

bool InterceptBallProvider::checkKickType(KickInfo::KickType kickType)
{
  return kickType == KickInfo::walkForwardsLeftLong ||
         kickType == KickInfo::walkForwardsRightLong ||
         kickType == KickInfo::walkForwardsLeftAlternative ||
         kickType == KickInfo::walkForwardsRightAlternative;
}

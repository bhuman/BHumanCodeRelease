/**
 * @file InterceptBallProviderDBHC.cpp
 *
 * This file implements a module that intercepts the ball.
 * This version was ONLY used in the DBHC.
 *
 * @author Philip Reichenberg
 */

#include "InterceptBallProviderDBHC.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Debugging/Annotation.h"

MAKE_MODULE(InterceptBallProviderDBHC);

void InterceptBallProviderDBHC::update(InterceptBallGenerator& theInterceptBallGenerator)
{
  if(!fieldBorderBottomRight)
  {
    fieldBorderBottomRight = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline);
    fieldBorderTopLeft = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
  }

  theInterceptBallGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    // Determine which velocity shall be used
    Vector2f useBallVelocity = motionRequest.ballEstimate.velocity;
    if(motionRequest.ballEstimate.velocity.squaredNorm() < 1.f)
    {
      const Vector2f ballDirection = theFieldBall.interceptedEndPositionRelative - motionRequest.ballEstimate.position;
      ASSERT(ballDirection.squaredNorm() != 0.f);

      // Something is broken, walk in place until behavior functions normally again
      if(ballDirection.squaredNorm() == 0.f)
      {
        ANNOTATION("InterceptBallProviderDBHC", "No interception possible!");
        return theWalkGenerator.createPhase(Pose2f(0.f, 0.001f, 0.f), lastPhase, 0.f);
      }

      useBallVelocity = ballDirection.normalized(BallPhysics::velocityForDistance(ballDirection.norm(), theBallSpecification.friction));
    }

    // Get transformation matrix from robotpose-relative coordinates to start-of-next-walkstep-relative coordinates
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, motionRequest.ballEstimate.position);
    const Pose3f supportInTorso3D = theTorsoMatrix * (isLeftPhase ? theRobotModel.soleRight : theRobotModel.soleLeft);
    const Pose2f supportInTorso(supportInTorso3D.rotation.getZAngle(), supportInTorso3D.translation.head<2>());
    const Pose2f hipOffset(0.f, 0.f, isLeftPhase ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset);
    const Pose2f scsCognition = hipOffset * supportInTorso.inverse() * theOdometryDataPreview.inverse() * motionRequest.odometryData;

    // Determine orth point
    const Geometry::Line ballLine = Geometry::Line(scsCognition * motionRequest.ballEstimate.position, useBallVelocity.rotated(scsCognition.rotation));
    Vector2f orth = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, Vector2f(bestHitPoint, 0.f));

    // Orth point is behind us or too much forward. Shift it to get correct y-value, so we can primary intercept with side steps
    if(orth.x() < 0 || orth.x() > maxXStep.max)
    {
      const float b = orth.x() < 0 ? -orth.x() : -(orth.x() - maxXStep.max);

      Angle alpha = ballLine.direction.angle();

      if(std::abs(alpha) > 90_deg)  // mirroring
        alpha = Angle::normalize(alpha + 180_deg);

      const float a = std::tan(alpha) * b;

      orth += Vector2f(b, a);
    }
    Vector2f orthForDistance = orth;
    if(orthForDistance.x() < 160.f)
    {
      const float b = 160.f - orthForDistance.x();

      Angle alpha = ballLine.direction.angle();

      if(std::abs(alpha) > 90_deg)   // mirroring
        alpha = Angle::normalize(alpha + 180_deg);

      const float a = std::tan(alpha) * b;

      orthForDistance += Vector2f(b, a);
    }

    // Get time, ball roll angle and size of shifted contact width
    const float timeForDistance = BallPhysics::timeForDistance(useBallVelocity, (orthForDistance - ballLine.base).norm(), theBallSpecification.friction);
    const Angle correctionAngle = Angle::normalize(ballLine.direction.angle() + 180_deg); // 360deg is one circle, half is 180deg. The direction is just mirrored
    const float yShiftClipped = maxYShift.limit(-Vector2f(theRobotDimensions.footLength, 0.f).rotate(correctionAngle).y());

    const Vector2f soleContact(0.f, yLegIntercept() + yShiftClipped);
    Pose2f walkPose(0_deg, orth - soleContact);

    // Ball is rolling at a steep angle -> rotate to allow for better sidewalking
    if(std::abs(ballLine.direction.angle()) < maxBallRollAngle)
      walkPose.rotation = theWalkGenerator.getRotationRange(isLeftPhase, motionRequest.walkSpeed).clamped(ballLine.base.y() > 0.f ? maxRotationStep : -maxRotationStep);

    // Clip rotation in case the wrong foot is the swing foot and we got not much time left
    if(walkPose.rotation != 0.f &&
       !((ballLine.base.squaredNorm() > sqr(minBallDistanceAlwaysRotate) && timeForDistance > minTimeForBallAlwaysRotate) ||
         (walkPose.translation.y() > 0.f) != isLeftPhase))
      walkPose.rotation = theWalkGenerator.getStepRotationRange(isLeftPhase, motionRequest.walkSpeed, walkPose.translation, false, lastPhase, true).limit(walkPose.rotation);

    if(timeForDistance < 0.6f)
    {
      // We got no time more. So only do the side walk if:
      // a) the ball would roll out of the field anyway
      // b) kicking the ball to the side would not kick it out of the field -> worth the risk
      const Vector2f ballEndPosition = scsCognition * BallPhysics::getEndPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity, theBallSpecification.friction);

      const float ballDistance = distanceBallEndToFieldBorderSquared(scsCognition, ballLine.base, ballEndPosition, ballLine.direction);

      if(ballDistance > 0.f)
        walkPose.translation.y() = 0.f;

      walkPose.translation.x() = maxXStep.min;
    }

    // wrong foot -> can't execute side step
    if(isLeftPhase == (walkPose.translation.y() < 0.f))
      walkPose.translation.y() = 0.f;

    std::vector<Vector2f> translationPolygon;
    std::vector<Vector2f> translationPolygonNoCenter;
    theWalkGenerator.getTranslationPolygon(isLeftPhase, walkPose.rotation, lastPhase, motionRequest.walkSpeed, translationPolygon, translationPolygonNoCenter, false, true);

    ASSERT(translationPolygon.size() > 0);
    const Vector2f& firstEdge = translationPolygon[0];
    if(std::abs(walkPose.translation.y()) >= std::abs(firstEdge.y()))
    {
      const float sign = static_cast<float>(sgnPos(walkPose.translation.y()));
      walkPose.translation.y() = firstEdge.y() * sign;
    }
    // Some optimization, to allow big side steps. We do not follow the shortest path as it does not matter here.
    // Clip walk step into allowed step polygon
    if(walkPose.translation.x() != 0.f && !Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), walkPose.translation))
    {
      Vector2f intersectionPoint;
      VERIFY(Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, walkPose.translation.y() * 0.99f), walkPose.translation), intersectionPoint));
      walkPose.translation = intersectionPoint;
    }

    return theWalkGenerator.createPhase(walkPose, lastPhase, 0.f);
  };
}

float InterceptBallProviderDBHC::yLegIntercept()
{
  return mapToRange(theRobotPose.translation.y(), -1000.f, 1000.f, -theRobotDimensions.yHipOffset, theRobotDimensions.yHipOffset);
}

float InterceptBallProviderDBHC::distanceBallEndToFieldBorderSquared(const Pose2f& scsCognition, const Vector2f& ballPosition, const Vector2f& ballEndPosition, const Vector2f& ballVelocity)
{
  const Pose2f relativeToField = scsCognition * theRobotPose;
  const Vector2f ballInField = relativeToField * ballPosition;
  const Vector2f ballVelInField = relativeToField * ballVelocity;
  const Vector2f ballEndInField = relativeToField * (ballEndPosition + (ballEndPosition - ballPosition).normalized(500.f));
  if(!theFieldDimensions.isInsideField(ballEndInField))
    return 0.f;

  Vector2f p1, p2;
  if(!Geometry::getIntersectionPointsOfLineAndRectangle(fieldBorderBottomRight.value(), fieldBorderTopLeft.value(), Geometry::Line(ballInField, ballVelInField), p1, p2))
    return 0.f;

  if(std::abs((p2 - ballInField).angle() - ballVelInField.angle()) < 0.1_deg)
    p1 = p2;

  return (p1 - ballEndInField).squaredNorm();
}

float InterceptBallProviderDBHC::distanceBallToFieldBorderSideKick(const Pose2f& scsCognition, const std::vector<Angle>& directions)
{
  const Pose2f relativeToField = scsCognition * theRobotPose;
  std::vector<float> distances;
  Vector2f p1, p2;
  for(const Angle a : directions)
  {
    const Angle direction = Angle::normalize(relativeToField.rotation + a);
    if(!Geometry::getIntersectionPointsOfLineAndRectangle(fieldBorderBottomRight.value(), fieldBorderTopLeft.value(), Geometry::Line(relativeToField.translation, Vector2f::polar(1.f, direction)), p1, p2))
      distances.push_back(0.f);
    else
    {
      if(std::abs((p2 - relativeToField.translation).angle() - direction) < 0.1_deg)
        p1 = p2;
      distances.push_back((p1 - relativeToField.translation).norm());
    }
  }

  return *std::min_element(distances.begin(), distances.end());
}

/**
 * @file FieldInterceptBallProvider.cpp
 *
 * Provides information about intercepting the ball for the Behavior
 *
 * @author Philip Reichenberg
 */

#include "FieldInterceptBallProvider.h"
#include "Math/Geometry.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(FieldInterceptBallProvider);

void FieldInterceptBallProvider::update(FieldInterceptBall& theFieldInterceptBall)
{
  calculateInterceptedBallEndPosition(theFieldInterceptBall);
}

void FieldInterceptBallProvider::checkIfBallIsPassingOwnXAxis(Vector2f& intersectionPositionWithOwnXAxis, float& timeUntilIntersectsOwnXAxis, const Vector2f& ballPosition, const Vector2f& ballVelocity, const bool distanceCheck = true)
{
  intersectionPositionWithOwnXAxis = Vector2f::Zero();
  timeUntilIntersectsOwnXAxis = std::numeric_limits<float>::max();

  if(ballVelocity.y() == 0.f || ballVelocity.y() * ballPosition.y() > 0.f || ballPosition.x() < 0.f) // Ball does not move or moves away
    return;
  Vector2f intersection;
  if(Geometry::getIntersectionOfLines(Geometry::Line(ballPosition, ballVelocity),
                                      Geometry::Line(Vector2f(0.f, 0.f), Vector2f(1.f, 0.f)), intersection))
  {
    const float distanceToIntersection = (intersection - ballPosition).norm();
    const float distanceToEndPosition = (theFieldBall.endPositionRelative - ballPosition).norm();
    // Is the ball fast enough to reach the intersection point?
    if(!distanceCheck || distanceToIntersection < distanceToEndPosition)
    {
      intersectionPositionWithOwnXAxis = intersection;
      timeUntilIntersectsOwnXAxis = BallPhysics::timeForDistance(ballVelocity, distanceToIntersection, theBallSpecification);
    }
  }
}

void FieldInterceptBallProvider::checkIfBallIsPassingOwnYAxis(Vector2f& intersectionPositionWithOwnYAxis, float& timeUntilIntersectsOwnYAxis, const Vector2f& ballPosition, const Vector2f& ballVelocity, const bool distanceCheck = true)
{
  intersectionPositionWithOwnYAxis = Vector2f::Zero();
  timeUntilIntersectsOwnYAxis = std::numeric_limits<float>::max();

  if(ballVelocity.x() >= 0.f || ballPosition.x() < 0.f) // Ball does not move or moves away
    return;
  Vector2f intersection;
  if(Geometry::getIntersectionOfLines(Geometry::Line(ballPosition, ballVelocity),
                                      Geometry::Line(Vector2f(0.f, 0.f), Vector2f(0.f, 1.f)), intersection))
  {
    const float distanceToIntersection = (intersection - ballPosition).norm();
    const float distanceToEndPosition = (theFieldBall.endPositionRelative - ballPosition).norm();
    // Is the ball fast enough to reach the intersection point?
    if(!distanceCheck || distanceToIntersection < distanceToEndPosition)
    {
      intersectionPositionWithOwnYAxis = intersection;
      timeUntilIntersectsOwnYAxis = BallPhysics::timeForDistance(ballVelocity, distanceToIntersection, theBallSpecification);
    }
  }
}

void FieldInterceptBallProvider::calculateInterceptedBallEndPosition(FieldInterceptBall& theFieldInterceptBall)
{
  // TODO: Walking speed should be considered
  // Decide whether an intersection should be done.

  const float ballDistance = theFieldBall.positionRelative.norm();
  Vector2f ballPosition = theFieldBall.positionRelative;
  Vector2f ballVelocity = theFieldBall.velocityRelative;
  Vector2f endPosition = theFieldBall.endPositionRelative;
  bool isRollingTowardsOpponentGoal = theFieldBall.isRollingTowardsOpponentGoal;
  checkIfBallIsPassingOwnYAxis(theFieldInterceptBall.intersectionPositionWithOwnYAxis, theFieldInterceptBall.timeUntilIntersectsOwnYAxis, ballPosition, ballVelocity);
  checkIfBallIsPassingOwnXAxis(theFieldInterceptBall.intersectionPositionWithOwnXAxis, theFieldInterceptBall.timeUntilIntersectsOwnXAxis, ballPosition, ballVelocity);

  if(!theFieldInterceptBall.interceptBall)
  {
    const bool generalIntersectionCondition = theFieldBall.ballWasSeen(ballSeenTime.min) &&
                                              (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
                                              !isRollingTowardsOpponentGoal &&
                                              ballDistance < 5000.f;

    const Angle ballAngle = ballPosition.angle();
    const float minBallPosX = mapToRange(std::abs(ballAngle), static_cast<float>(ballAngleRange.min), static_cast<float>(ballAngleRange.max), minBallDistance.max, minBallDistance.min);

    const bool yIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                        between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.0f) &&
                                        endPosition.x() < ballEndPositionX.min && ballPosition.x() > minBallPosX;

    const bool xIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnXAxis.x(), 0, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                        between<float>(theFieldInterceptBall.timeUntilIntersectsOwnXAxis, 0.1f, 3.0f) &&
                                        // ball rolls negative -> end must be further away (lower) than -min, else must be further away (higher) than +min
                                        (ballVelocity.y() < 0.f ? endPosition.y() < -ballEndPositionY.min : endPosition.y() > ballEndPositionY.max);

    theFieldInterceptBall.interceptBall = generalIntersectionCondition && (yIntersectionCondition || (xIntersectionCondition && useXAxisIntersection));
  }
  else
  {
    const bool generalIntersectionCondition = theFieldBall.ballWasSeen(ballSeenTime.max) &&
                                              !isRollingTowardsOpponentGoal;

    const bool yIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis) &&
                                        between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 4.0f) &&
                                        endPosition.x() < ballEndPositionX.max;

    const bool xIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnXAxis.x(), 0 - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis) &&
                                        between<float>(theFieldInterceptBall.timeUntilIntersectsOwnXAxis, 0.1f, 4.0f) &&
                                        // ball rolls negative -> end must be further away (lower) than -min, else must be further away (higher) than +min
                                        (ballVelocity.y() < 0.f ? endPosition.y() < -ballEndPositionY.max : endPosition.y() > ballEndPositionY.min);

    theFieldInterceptBall.interceptBall = generalIntersectionCondition && (yIntersectionCondition || (xIntersectionCondition && useXAxisIntersection));
  }
  if(theFieldInterceptBall.interceptBall)
  {
    theFieldInterceptBall.predictedInterceptBall = false;
  }
  else
  {
    if(!theFieldInterceptBall.predictedInterceptBall)
    {
      const Angle ballAngle = ballPosition.angle();
      const float minBallPosX = mapToRange(std::abs(ballAngle), static_cast<float>(ballAngleRange.min), static_cast<float>(ballAngleRange.max), minBallDistance.max, minBallDistance.min);

      const bool generalIntersectionCondition = theFieldBall.ballWasSeen(ballSeenTime.min) &&
                                                (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
                                                !isRollingTowardsOpponentGoal &&
                                                ballDistance < 5000.f &&
                                                endPosition.x() < 1000.f;

      const bool yIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                          ballPosition.x() > minBallPosX &&
                                          (between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.0f) || ((endPosition - ballPosition).squaredNorm() > sqr(1000.f) && theFieldInterceptBall.intersectionPositionWithOwnYAxis != Vector2f::Zero()));

      theFieldInterceptBall.predictedInterceptBall = generalIntersectionCondition && yIntersectionCondition;
    }
    else
    {
      const bool generalIntersectionCondition = theFieldBall.ballWasSeen(ballSeenTime.max) &&
                                                !isRollingTowardsOpponentGoal &&
                                                endPosition.x() < 1200.f;

      const bool yIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis) &&
                                          (between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 4.0f) || ((endPosition - ballPosition).squaredNorm() > sqr(500.f) && theFieldInterceptBall.intersectionPositionWithOwnYAxis != Vector2f::Zero()));

      theFieldInterceptBall.predictedInterceptBall = generalIntersectionCondition && yIntersectionCondition;
    }
  }

  if(ballVelocity == Vector2f::Zero())
    theFieldInterceptBall.interceptBall = false;

  // Use standard ball end position if the ball cannot be intercepted

  const bool hackyGeneralIntersectionCondition = theFieldBall.ballWasSeen(ballSeenTime.min) &&
                                                 (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
                                                 !isRollingTowardsOpponentGoal &&
                                                 ballDistance < 5000.f;

  const bool hackyYIntersectionCondition = between<float>(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                           between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.0f);

  if(!theFieldInterceptBall.interceptBall && !(hackyGeneralIntersectionCondition && hackyYIntersectionCondition))
  {
    // If close to the ball, prefer the current ball position.
    // Otherwise if the ball is rolling near to us, it could cause unnecessary striker switching
    // due to wrong or irrelevant ball end positions
    const float interpolationFactor = mapToRange(ballDistance, 500.f, 1500.f, 0.f, 1.f);
    theFieldInterceptBall.interceptedEndPositionRelative = interpolationFactor * theFieldBall.endPositionRelative + (1.f - interpolationFactor) * theFieldBall.positionRelative;
    theFieldInterceptBall.interceptedEndPositionOnField = interpolationFactor * theFieldBall.endPositionOnField + (1.f - interpolationFactor) * theFieldBall.positionOnField;
    static_cast<void>(theFieldDimensions.clipToCarpet(theFieldInterceptBall.interceptedEndPositionOnField));
    return;
  }

  // Ball trajectory
  const Geometry::Line ballLine = Geometry::Line(ballPosition, ballVelocity);

  // Calculate the interception point by taking the orthogonal intersection between the ball trajectory and the kick leg (with ballOffset)
  const Vector2f orth = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, Vector2f(0.f, 0.f));

  theFieldInterceptBall.interceptedEndPositionRelative = orth;
  theFieldInterceptBall.interceptedEndPositionOnField = theRobotPose * theFieldInterceptBall.interceptedEndPositionRelative;
}

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
  // Risky ball estimate shall not be used as goal keeper or in penalty kick|shootout
  useRiskyBallEstimateAllowed = useRiskyBallEstimate && !theGameState.isGoalkeeper() && !theGameState.isPenaltyShootout() && !theGameState.isPenaltyKick();
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
      timeUntilIntersectsOwnXAxis = BallPhysics::timeForDistance(ballVelocity, distanceToIntersection, theBallSpecification.friction);
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
      timeUntilIntersectsOwnYAxis = BallPhysics::timeForDistance(ballVelocity, distanceToIntersection, theBallSpecification.friction);
    }
  }
}

void FieldInterceptBallProvider::calculateInterceptedBallEndPosition(FieldInterceptBall& theFieldInterceptBall)
{
  // TODO: Walking speed should be considered
  // Decide whether an intersection should be done.

  // Interpolate between the normal ball estimate and the risky estimate
  const float ballDistance = theFieldBall.positionRelative.norm();
  Vector2f ballPosition = theFieldBall.positionRelative;
  Vector2f ballVelocity = theFieldBall.velocityRelative;
  float timeUntilIntersectsOwnYAxis = theFieldInterceptBall.timeUntilIntersectsOwnYAxis;
  float timeUntilIntersectsOwnXAxis = theFieldInterceptBall.timeUntilIntersectsOwnXAxis;
  Vector2f& intersectionPositionWithOwnYAxis = theFieldInterceptBall.intersectionPositionWithOwnYAxis;
  Vector2f& intersectionPositionWithOwnXAxis = theFieldInterceptBall.intersectionPositionWithOwnXAxis;
  Vector2f endPosition = theFieldBall.endPositionRelative;
  bool isRollingTowardsOpponentGoal = theFieldBall.isRollingTowardsOpponentGoal;
  // The ball must be a min distance away and the risky estimate must have a higher velocity
  if(!isRollingTowardsOpponentGoal && useRiskyBallEstimateAllowed && theBallModel.riskyMovingEstimateIsValid && ballDistance > interpolateRiskyBallEstimateRange.min &&
     ballVelocity.squaredNorm() < theBallModel.riskyMovingEstimate.velocity.squaredNorm())
  {
    const float ballDistanceScaling = Rangef::ZeroOneRange().limit((ballDistance - interpolateRiskyBallEstimateRange.min) / (interpolateRiskyBallEstimateRange.max - interpolateRiskyBallEstimateRange.min));
    ballPosition = ballPosition * (1.f - ballDistanceScaling) + theBallModel.riskyMovingEstimate.position * ballDistanceScaling;
    ballVelocity = ballVelocity * (1.f - ballDistanceScaling) + theBallModel.riskyMovingEstimate.velocity * ballDistanceScaling;
    endPosition = BallPhysics::getEndPosition(ballPosition, ballVelocity, theBallSpecification.friction);
  }
  checkIfBallIsPassingOwnYAxis(theFieldInterceptBall.intersectionPositionWithOwnYAxis, theFieldInterceptBall.timeUntilIntersectsOwnYAxis, ballPosition, ballVelocity);
  checkIfBallIsPassingOwnXAxis(theFieldInterceptBall.intersectionPositionWithOwnXAxis, theFieldInterceptBall.timeUntilIntersectsOwnXAxis, ballPosition, ballVelocity);

  if(!theFieldInterceptBall.interceptBall)
  {
    const bool generalIntersectionCondition = theFieldBall.ballWasSeen(100) &&
                                              (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
                                              !isRollingTowardsOpponentGoal &&
                                              ballDistance < 5000.f &&
                                              endPosition.x() < 200.f;

    const bool yIntersectionCondition = between<float>(intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                        between<float>(timeUntilIntersectsOwnYAxis, 0.1f, 3.0f);

    const bool xIntersectionCondition = between<float>(intersectionPositionWithOwnXAxis.x(), 0, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                        between<float>(timeUntilIntersectsOwnXAxis, 0.1f, 3.0f);

    theFieldInterceptBall.interceptBall = generalIntersectionCondition && (yIntersectionCondition || (xIntersectionCondition && useXAxisIntersection));
  }
  else
  {
    const bool generalIntersectionCondition = !theFieldBall.ballWasSeen(150) ||
                                              isRollingTowardsOpponentGoal ||
                                              endPosition.x() > 300.f;

    const bool yIntersectionCondition = !(between<float>(intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis) &&
                                          between<float>(timeUntilIntersectsOwnYAxis, 0.1f, 3.0f));

    const bool xIntersectionCondition = !(between<float>(intersectionPositionWithOwnXAxis.x(), 0 - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis) &&
                                          between<float>(timeUntilIntersectsOwnXAxis, 0.1f, 3.0f));

    theFieldInterceptBall.interceptBall = !((yIntersectionCondition && (xIntersectionCondition || !useXAxisIntersection)) || generalIntersectionCondition);
  }
  if(theFieldInterceptBall.interceptBall)
  {
    theFieldInterceptBall.lastInterceptBall = theFrameInfo.time;
    theFieldInterceptBall.predictedInterceptBall = false;
  }
  else
  {
    float timeUntilIntersectsOwnYAxis;
    checkIfBallIsPassingOwnYAxis(intersectionPositionWithOwnYAxis, timeUntilIntersectsOwnYAxis, ballPosition, ballVelocity, false);

    if(!theFieldInterceptBall.predictedInterceptBall)
    {
      const bool generalIntersectionCondition = theFieldBall.ballWasSeen(100) &&
                                                (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
                                                !isRollingTowardsOpponentGoal &&
                                                ballDistance < 5000.f &&
                                                endPosition.x() < 1000.f;

      const bool yIntersectionCondition = between<float>(intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                          (between<float>(timeUntilIntersectsOwnYAxis, 0.1f, 3.0f) || ((endPosition - ballPosition).squaredNorm() > sqr(1000.f) && intersectionPositionWithOwnYAxis != Vector2f::Zero()));

      theFieldInterceptBall.predictedInterceptBall = generalIntersectionCondition && yIntersectionCondition;
    }
    else
    {
      const bool generalIntersectionCondition = !theFieldBall.ballWasSeen(150) ||
                                                isRollingTowardsOpponentGoal ||
                                                endPosition.x() > 1200.f;

      const bool yIntersectionCondition = !(between<float>(intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis) &&
                                            (between<float>(timeUntilIntersectsOwnYAxis, 0.1f, 3.0f) || ((endPosition - ballPosition).squaredNorm() > sqr(500.f) && intersectionPositionWithOwnYAxis != Vector2f::Zero())));

      theFieldInterceptBall.predictedInterceptBall = !(yIntersectionCondition || generalIntersectionCondition);
    }
  }

  if(ballVelocity == Vector2f::Zero())
    theFieldInterceptBall.interceptBall = false;

  // Use standard ball end position if the ball cannot be intercepted
  if(!theFieldInterceptBall.interceptBall)
  {
    theFieldInterceptBall.interceptedEndPositionRelative = theFieldBall.endPositionRelative;
    theFieldInterceptBall.interceptedEndPositionOnField = theFieldBall.endPositionOnField;
    return;
  }

  // Ball trajectory
  const Geometry::Line ballLine = Geometry::Line(ballPosition, ballVelocity);

  // Choose the kick leg depending on which side the ball is coming from
  // TODO eval if taking the normal intercetion point would be better for the behavior
  const KickInfo::KickType kickType = ballPosition.y() < 0 ? KickInfo::walkForwardsRight : KickInfo::walkForwardsLeft;

  // Calculate the interception point by taking the orthogonal intersection between the ball trajectory and the kick leg (with ballOffset)
  Vector2f orth = Geometry::getOrthogonalProjectionOfPointOnLine(ballLine, -(theKickInfo[kickType].ballOffset));

  Pose2f kickPose = Pose2f(0, orth);
  kickPose.translate(theKickInfo[kickType].ballOffset);

  // If intersection point is behind the robot
  if(kickPose.translation.x() < 0)
  {
    const float b = std::abs(orth.x() + theKickInfo[kickType].ballOffset.x());

    Angle alpha = ballLine.direction.angle();

    if(std::abs(alpha) > 90_deg)
      alpha = Angle::normalize(alpha + 180_deg);

    const float a = std::tan(alpha) * b;

    orth += Vector2f(b, a);
  }
  theFieldInterceptBall.interceptedEndPositionRelative = orth;
  theFieldInterceptBall.interceptedEndPositionOnField = theRobotPose * theFieldInterceptBall.interceptedEndPositionRelative;
}

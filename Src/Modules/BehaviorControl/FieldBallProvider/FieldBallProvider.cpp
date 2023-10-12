/**
 * @file BehaviorControl/FieldBallProvider/FieldBallProvider.h
 *
 * Provides the FieldBall for the Behavior
 *
 * @author Tim Laue
 */

#include "FieldBallProvider.h"
#include "Math/Geometry.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(FieldBallProvider);

void FieldBallProvider::update(FieldBall& fieldBall)
{
  fieldBall.positionRelative        = theBallModel.estimate.position;
  fieldBall.positionOnField         = theRobotPose * fieldBall.positionRelative;
  fieldBall.positionOnFieldClipped  = fieldBall.positionOnField;
  theFieldDimensions.clipToField(fieldBall.positionOnFieldClipped);
  fieldBall.positionRelativeClipped = theRobotPose.inverse() * fieldBall.positionOnFieldClipped;
  fieldBall.endPositionRelative     = BallPhysics::getEndPosition(theBallModel.estimate.position, theBallModel.estimate.velocity, theBallSpecification.friction);
  fieldBall.endPositionOnField      = theRobotPose * fieldBall.endPositionRelative;

  fieldBall.teammatesBallIsValid = theTeammatesBallModel.isValid;
  if(theTeammatesBallModel.isValid)
  {
    fieldBall.teamPositionOnField     = theTeammatesBallModel.position;
    fieldBall.teamPositionRelative    = theRobotPose.inverse() * theTeammatesBallModel.position;
    fieldBall.teamEndPositionOnField  = BallPhysics::getEndPosition(theTeammatesBallModel.position, theTeammatesBallModel.velocity, theBallSpecification.friction);
    fieldBall.teamEndPositionRelative = theRobotPose.inverse() * fieldBall.teamEndPositionOnField;
  }

  fieldBall.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  fieldBall.timeSinceBallDisappeared = theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared);

  fieldBall.teammatesBallNewerThanOwnBall = theTeammatesBallModel.newerThanOwnBall;

  checkIfBallIsRollingTowardsAGoal(fieldBall.isRollingTowardsOpponentGoal, fieldBall.isRollingTowardsOwnGoal,
                                   fieldBall.positionOnField, theBallModel.estimate.velocity);
  checkIfBallIsPassingOwnYAxis(fieldBall.intersectionPositionWithOwnYAxis, fieldBall.timeUntilIntersectsOwnYAxis, theBallModel.estimate);
  checkIfBallIsPassingOwnXAxis(fieldBall.intersectionPositionWithOwnXAxis, fieldBall.timeUntilIntersectsOwnXAxis, theBallModel.estimate);
  checkIfBallIsInsideOwnPenaltyArea(fieldBall);
  checkBallPositionIsConsistentWithGameState(fieldBall, theGameState);
  calculateInterceptedBallEndPosition(fieldBall);
}

void FieldBallProvider::checkIfBallIsRollingTowardsAGoal(bool& isRollingTowardsOpponentGoal, bool& isRollingTowardsOwnGoal,
                                                         const Vector2f& positionOnField, const Vector2f& velocity)
{
  isRollingTowardsOpponentGoal = false;
  isRollingTowardsOwnGoal = false;
  if(theBallModel.estimate.velocity.norm() == 0.f)
    return;
  const Vector2f bPosField = positionOnField;
  const Vector2f bVelField = velocity.rotated(theRobotPose.rotation);
  const Vector2f leftOppPost(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal + opponentGoalpostYOffset);
  const Vector2f rightOppPost(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal - opponentGoalpostYOffset);
  const Vector2f leftOwnPost(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal + ownGoalpostYOffset);
  const Vector2f rightOwnPost(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal - ownGoalpostYOffset);

  // Check, if ball is rolling TOWARDS (this is why we multiply the velocity) the expanded opponent goal:
  isRollingTowardsOpponentGoal = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOppPost, rightOppPost);
  isRollingTowardsOwnGoal      = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOwnPost, rightOwnPost);
}

void FieldBallProvider::checkIfBallIsPassingOwnXAxis(Vector2f& intersectionPositionWithOwnXAxis, float& timeUntilIntersectsOwnXAxis, const BallState& estimate)
{
  intersectionPositionWithOwnXAxis = Vector2f::Zero();
  timeUntilIntersectsOwnXAxis = std::numeric_limits<float>::max();

  const Vector2f& ballPosRel = estimate.position;
  const Vector2f& ballVelRel = estimate.velocity;
  if(ballVelRel.x() >= 0.f || ballPosRel.x() < 0.f) // Ball does not move or moves away
    return;
  Vector2f intersection;
  if(Geometry::getIntersectionOfLines(Geometry::Line(ballPosRel, ballVelRel),
                                      Geometry::Line(Vector2f(0.f, 0.f), Vector2f(1.f, 0.f)), intersection))
  {
    const float distanceToIntersection = (intersection - ballPosRel).norm();
    Vector2f ballEndPosition = BallPhysics::getEndPosition(ballPosRel, ballVelRel, theBallSpecification.friction);
    const float distanceToEndPosition = (ballEndPosition - ballPosRel).norm();
    // Is the ball fast enough to reach the intersection point?
    if(distanceToIntersection < distanceToEndPosition)
    {
      intersectionPositionWithOwnXAxis = intersection;
      timeUntilIntersectsOwnXAxis = BallPhysics::timeForDistance(ballVelRel, distanceToIntersection, theBallSpecification.friction);
    }
  }
}

void FieldBallProvider::checkIfBallIsPassingOwnYAxis(Vector2f& intersectionPositionWithOwnYAxis, float& timeUntilIntersectsOwnYAxis, const BallState& estimate)
{
  intersectionPositionWithOwnYAxis = Vector2f::Zero();
  timeUntilIntersectsOwnYAxis = std::numeric_limits<float>::max();

  const Vector2f& ballPosRel = estimate.position;
  const Vector2f& ballVelRel = estimate.velocity;
  if(ballVelRel.x() >= 0.f || ballPosRel.x() < 0.f) // Ball does not move or moves away
    return;
  Vector2f intersection;
  if(Geometry::getIntersectionOfLines(Geometry::Line(ballPosRel, ballVelRel),
                                      Geometry::Line(Vector2f(0.f, 0.f), Vector2f(0.f, 1.f)), intersection))
  {
    const float distanceToIntersection = (intersection - ballPosRel).norm();
    Vector2f ballEndPosition = BallPhysics::getEndPosition(ballPosRel, ballVelRel, theBallSpecification.friction);
    const float distanceToEndPosition = (ballEndPosition - ballPosRel).norm();
    // Is the ball fast enough to reach the intersection point?
    if(distanceToIntersection < distanceToEndPosition)
    {
      intersectionPositionWithOwnYAxis = intersection;
      timeUntilIntersectsOwnYAxis = BallPhysics::timeForDistance(ballVelRel, distanceToIntersection, theBallSpecification.friction);
    }
  }
}

void FieldBallProvider::checkIfBallIsInsideOwnPenaltyArea(FieldBall& fieldBall)
{
  fieldBall.isInsideOwnPenaltyArea = false;
  fieldBall.distanceToOwnPenaltyArea = -1.f;
  if(!fieldBall.ballWasSeen() && !theTeammatesBallModel.isValid)
    return;
  const Vector2f& bp = fieldBall.recentBallPositionOnField();
  fieldBall.isInsideOwnPenaltyArea = bp.x() < theFieldDimensions.xPosOwnPenaltyArea &&
                                     std::abs(bp.y()) < theFieldDimensions.yPosLeftPenaltyArea;
  if(fieldBall.isInsideOwnPenaltyArea)
  {
    fieldBall.distanceToOwnPenaltyArea = 0.f;
  }
  else
  {
    float dx = std::max(0.f, bp.x() - theFieldDimensions.xPosOwnPenaltyArea);
    float dy = std::max(0.f, std::abs(bp.y()) - theFieldDimensions.yPosLeftPenaltyArea);
    if(dx > 0.f && dy == 0.f) // ball in front of penalty area
      fieldBall.distanceToOwnPenaltyArea = dx;
    else if(dx == 0.f && dy >= 0.f) // ball is next to penalty area
      fieldBall.distanceToOwnPenaltyArea = dy;
    else if(bp.y() > 0.f) // ball is left front of penalty area
      fieldBall.distanceToOwnPenaltyArea = (Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea) - bp).norm();
    else // ball is right front of penalty area
      fieldBall.distanceToOwnPenaltyArea = (Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea) - bp).norm();
  }
}

void FieldBallProvider::calculateInterceptedBallEndPosition(FieldBall& fieldBall)
{
  // TODO: Walking speed should be considered
  // Decide whether an intersection should be done.

  // Interpolate between the normal ball estimate and the risky estimate
  const float ballDistance = fieldBall.positionRelative.norm();
  BallState estimate = theBallModel.estimate;
  float timeUntilIntersectsOwnYAxis = fieldBall.timeUntilIntersectsOwnYAxis;
  float timeUntilIntersectsOwnXAxis = fieldBall.timeUntilIntersectsOwnXAxis;
  Vector2f intersectionPositionWithOwnYAxis = fieldBall.intersectionPositionWithOwnYAxis;
  Vector2f intersectionPositionWithOwnXAxis = fieldBall.intersectionPositionWithOwnXAxis;
  Vector2f endPosition = fieldBall.endPositionRelative;
  bool isRollingTowardsOpponentGoal = fieldBall.isRollingTowardsOpponentGoal;
  bool isRollingTowardsOwnGoal = fieldBall.isRollingTowardsOwnGoal;
  // The ball must be a min distance away and the risky estimate must have a higher velocity
  if(useRiskyBallEstimate && theBallModel.riskyMovingEstimateIsValid && ballDistance > interpolateRiskyBallEstimateRange.min &&
     estimate.velocity.squaredNorm() < theBallModel.riskyMovingEstimate.velocity.squaredNorm())
  {
    const float ballDistanceScaling = Rangef::ZeroOneRange().limit((ballDistance - interpolateRiskyBallEstimateRange.min) / (interpolateRiskyBallEstimateRange.max - interpolateRiskyBallEstimateRange.min));
    estimate.position = estimate.position * (1.f - ballDistanceScaling) + theBallModel.riskyMovingEstimate.position * ballDistanceScaling;
    estimate.velocity = estimate.velocity * (1.f - ballDistanceScaling) + theBallModel.riskyMovingEstimate.velocity * ballDistanceScaling;
    endPosition = BallPhysics::getEndPosition(estimate.position, estimate.velocity, theBallSpecification.friction);
    checkIfBallIsPassingOwnYAxis(intersectionPositionWithOwnYAxis, timeUntilIntersectsOwnYAxis, estimate);
    checkIfBallIsPassingOwnXAxis(intersectionPositionWithOwnXAxis, timeUntilIntersectsOwnXAxis, estimate);
    checkIfBallIsRollingTowardsAGoal(isRollingTowardsOpponentGoal, isRollingTowardsOwnGoal,
                                     theRobotPose * estimate.position, estimate.velocity);
  }

  if(!fieldBall.interceptBall)
  {
    const bool generalIntersectionCondition = fieldBall.ballWasSeen(100) &&
                                              (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering) &&
                                              !isRollingTowardsOpponentGoal &&
                                              ballDistance < 5000.f &&
                                              endPosition.x() <= 200.f;

    const bool yIntersectionCondition = between<float>(intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                        between<float>(timeUntilIntersectsOwnYAxis, 0.1f, 3.0f);

    const bool xIntersectionCondition = between<float>(intersectionPositionWithOwnXAxis.x(), 0, theBehaviorParameters.ballCatchMaxWalkDistance) &&
                                        between<float>(timeUntilIntersectsOwnXAxis, 0.1f, 3.0f);

    fieldBall.interceptBall = generalIntersectionCondition && (yIntersectionCondition || (xIntersectionCondition && useXAxisIntersection));
  }
  else
  {
    const bool generalIntersectionCondition = !fieldBall.ballWasSeen(150) ||
                                              isRollingTowardsOpponentGoal ||
                                              endPosition.x() > 200.f;

    const bool yIntersectionCondition = !between<float>(intersectionPositionWithOwnYAxis.y(), -theBehaviorParameters.ballCatchMaxWalkDistance - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis);

    const bool xIntersectionCondition = !between<float>(intersectionPositionWithOwnXAxis.x(), 0 - ballInterceptionWidthHysteresis, theBehaviorParameters.ballCatchMaxWalkDistance + ballInterceptionWidthHysteresis);

    fieldBall.interceptBall = !((yIntersectionCondition && (xIntersectionCondition || !useXAxisIntersection)) || generalIntersectionCondition);
  }
  if(fieldBall.interceptBall)
    fieldBall.lastInterceptBall = theFrameInfo.time;

  if(estimate.velocity == Vector2f::Zero())
    fieldBall.interceptBall = false;

  // Use standard ball end position if the ball cannot be intercepted
  if(!fieldBall.interceptBall)
  {
    fieldBall.interceptedEndPositionRelative = fieldBall.endPositionRelative;
    fieldBall.interceptedEndPositionOnField = fieldBall.endPositionOnField;
    return;
  }

  // Ball trajectory
  const Geometry::Line ballLine = Geometry::Line(estimate.position, estimate.velocity);

  // Choose the kick leg depending on which side the ball is coming from
  const KickInfo::KickType kickType = estimate.position.y() < 0 ? KickInfo::walkForwardsRight : KickInfo::walkForwardsLeft;

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
  fieldBall.interceptedEndPositionRelative = orth;
  fieldBall.interceptedEndPositionOnField = theRobotPose * fieldBall.interceptedEndPositionRelative;
}

void FieldBallProvider::checkBallPositionIsConsistentWithGameState(FieldBall& fieldBall, const GameState& theGameState)
{
  if(theGameState.isKickOff())
  {
    Vector2f kickoffPosition = Vector2f(theFieldDimensions.xPosHalfWayLine, 0.f);
    fieldBall.ballPositionConsistentWithGameState = (fieldBall.positionOnField - kickoffPosition).squaredNorm() <= sqr(consistencyDistanceToDropInPosition);
    fieldBall.teamBallPositionConsistentWithGameState = (fieldBall.teamPositionOnField - kickoffPosition).squaredNorm() <=
                                                        sqr(consistencyDistanceToDropInPosition);
  }
  else if(theGameState.isCornerKick() || theGameState.isGoalKick())
  {
    Vector2f leftDropInPosition;
    Vector2f rightDropInPosition;
    if(theGameState.state == GameState::State::ownCornerKick)
    {
      leftDropInPosition = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
      rightDropInPosition = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline);
    }
    else if(theGameState.state == GameState::State::opponentCornerKick)
    {
      leftDropInPosition = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftSideline);
      rightDropInPosition = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightSideline);
    }
    else if(theGameState.state == GameState::State::ownGoalKick)
    {
      leftDropInPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea);
      rightDropInPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea);
    }
    else if(theGameState.state == GameState::State::opponentGoalKick)
    {
      leftDropInPosition = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea);
      rightDropInPosition = Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea);
    }
    float squaredDistanceToDropInLeft = (leftDropInPosition - fieldBall.positionOnField).squaredNorm();
    float squaredDistanceToDropInRight = (rightDropInPosition - fieldBall.positionOnField).squaredNorm();
    fieldBall.ballPositionConsistentWithGameState = std::min(squaredDistanceToDropInLeft, squaredDistanceToDropInRight) <= sqr(consistencyDistanceToDropInPosition);
    float teamSquaredDistanceLeft = (leftDropInPosition - fieldBall.teamPositionOnField).squaredNorm();
    float teamSquaredDistanceRight = (rightDropInPosition - fieldBall.teamPositionOnField).squaredNorm();
    fieldBall.teamBallPositionConsistentWithGameState = std::min(teamSquaredDistanceLeft, teamSquaredDistanceRight) <= sqr(consistencyDistanceToDropInPosition);
  }
  // no special handling for kick-in because the ball will be placed back near to where it rolled out -> position should be consistent for most kick-ins
  else if(theGameState.isPlaying())
  {
    fieldBall.ballPositionConsistentWithGameState = fieldBall.teamBallPositionConsistentWithGameState = true;
  }
  else
  {
    fieldBall.ballPositionConsistentWithGameState = fieldBall.teamBallPositionConsistentWithGameState = false;
  }
}

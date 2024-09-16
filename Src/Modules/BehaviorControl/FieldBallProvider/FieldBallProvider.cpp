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
  fieldBall.velocityRelative = theBallModel.estimate.velocity;
  fieldBall.velocityOnField = theRobotPose * fieldBall.velocityRelative;

  fieldBall.teammatesBallIsValid = theTeammatesBallModel.isValid;
  if(theTeammatesBallModel.isValid)
  {
    fieldBall.teamPositionOnField     = theTeammatesBallModel.position;
    fieldBall.teamPositionRelative    = theRobotPose.inverse() * theTeammatesBallModel.position;
    fieldBall.teamEndPositionOnField  = BallPhysics::getEndPosition(theTeammatesBallModel.position, theTeammatesBallModel.velocity, theBallSpecification.friction);
    fieldBall.teamEndPositionRelative = theRobotPose.inverse() * fieldBall.teamEndPositionOnField;
    fieldBall.teamVelocityOnField = theRobotPose.inverse() * fieldBall.teamVelocityRelative;
    fieldBall.teamVelocityRelative = theTeammatesBallModel.velocity;
  }

  fieldBall.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  fieldBall.timeSinceBallDisappeared = theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared);

  fieldBall.teammatesBallNewerThanOwnBall = theTeammatesBallModel.newerThanOwnBall;

  checkIfBallIsRollingTowardsAGoal(fieldBall.isRollingTowardsOpponentGoal, fieldBall.isRollingTowardsOwnGoal,
                                   fieldBall.positionOnField, theBallModel.estimate.velocity);
  checkIfBallIsInsideOwnPenaltyArea(fieldBall);
  checkBallPositionIsConsistentWithGameState(fieldBall, theGameState);
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
  const Vector2f leftOppPost(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoal + opponentGoalpostYOffset);
  const Vector2f rightOppPost(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoal - opponentGoalpostYOffset);
  const Vector2f leftOwnPost(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftGoal + ownGoalpostYOffset);
  const Vector2f rightOwnPost(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightGoal - ownGoalpostYOffset);

  // Check, if ball is rolling TOWARDS (this is why we multiply the velocity) the expanded opponent goal:
  isRollingTowardsOpponentGoal = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOppPost, rightOppPost);
  isRollingTowardsOwnGoal      = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOwnPost, rightOwnPost);
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

void FieldBallProvider::checkBallPositionIsConsistentWithGameState(FieldBall& fieldBall, const GameState& theGameState)
{
  if(theGameState.isKickOff())
  {
    Vector2f kickoffPosition = Vector2f(theFieldDimensions.xPosHalfwayLine, 0.f);
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
      leftDropInPosition = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftTouchline);
      rightDropInPosition = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightTouchline);
    }
    else if(theGameState.state == GameState::State::opponentCornerKick)
    {
      leftDropInPosition = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosLeftTouchline);
      rightDropInPosition = Vector2f(theFieldDimensions.xPosOwnGoalLine, theFieldDimensions.yPosRightTouchline);
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

/**
 * @file BehaviorControl/FieldBallProvider/FieldBallProvider.h
 *
 * Provides the FieldBall for the Behavior
 *
 * @author Tim Laue
 */

#include "FieldBallProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Modeling/BallPhysics.h"

MAKE_MODULE(FieldBallProvider, behaviorControl);

void FieldBallProvider::update(FieldBall& fieldBall)
{
  fieldBall.positionRelative        = theBallModel.estimate.position;
  fieldBall.positionOnField         = theRobotPose * fieldBall.positionRelative;
  fieldBall.positionOnFieldClipped  = fieldBall.positionOnField;
  theFieldDimensions.clipToField(fieldBall.positionOnFieldClipped);
  fieldBall.positionRelativeClipped = theRobotPose.inversePose * fieldBall.positionOnFieldClipped;
  fieldBall.endPositionRelative     = BallPhysics::getEndPosition(theBallModel.estimate.position, theBallModel.estimate.velocity, theBallSpecification.friction);
  fieldBall.endPositionOnField      = theRobotPose * fieldBall.endPositionRelative;

  fieldBall.teamPositionOnField     = theTeamBallModel.position;
  fieldBall.teamPositionRelative    = theRobotPose.inversePose * fieldBall.teamPositionOnField;
  fieldBall.teamEndPositionOnField  = BallPhysics::getEndPosition(theTeamBallModel.position, theTeamBallModel.velocity, theBallSpecification.friction);
  fieldBall.teamEndPositionRelative = theRobotPose.inversePose * fieldBall.teamEndPositionOnField;

  fieldBall.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  fieldBall.timeSinceBallDisappeared = theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared);
  fieldBall.timeSinceTeamBallWasValid = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastValid);

  checkIfBallIsRollingTowardsAGoal(fieldBall);
  checkIfBallIsPassingOwnYAxis(fieldBall);
  checkIfBallIsInsideOwnPenaltyArea(fieldBall);
}

void FieldBallProvider::checkIfBallIsRollingTowardsAGoal(FieldBall& fieldBall)
{
  fieldBall.isRollingTowardsOpponentGoal = false;
  fieldBall.isRollingTowardsOwnGoal = false;
  if(theBallModel.estimate.velocity.norm() == 0.f)
    return;
  const Vector2f bPosField = fieldBall.positionOnField;
  const Vector2f bVelField = theBallModel.estimate.velocity.rotated(theRobotPose.rotation);
  const Vector2f leftOppPost(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftGoal + opponentGoalpostYOffset);
  const Vector2f rightOppPost(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightGoal - opponentGoalpostYOffset);
  const Vector2f leftOwnPost(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal + ownGoalpostYOffset);
  const Vector2f rightOwnPost(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal - ownGoalpostYOffset);

  // Check, if ball is rolling TOWARDS (this is why we multiply the velocity) the expanded opponent goal:
  fieldBall.isRollingTowardsOpponentGoal = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOppPost, rightOppPost);
  fieldBall.isRollingTowardsOwnGoal      = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOwnPost, rightOwnPost);
}

void FieldBallProvider::checkIfBallIsPassingOwnYAxis(FieldBall& fieldBall)
{
  fieldBall.intersectionPositionWithOwnYAxis = Vector2f::Zero();
  fieldBall.timeUntilIntersectsOwnYAxis = std::numeric_limits<float>::max();

  const Vector2f& ballPosRel = theBallModel.estimate.position;
  const Vector2f& ballVelRel = theBallModel.estimate.velocity;
  if(ballVelRel.x() >= 0 || ballPosRel.x() < 0.f) // Ball does not move or moves away
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
      fieldBall.intersectionPositionWithOwnYAxis = intersection;
      fieldBall.timeUntilIntersectsOwnYAxis = BallPhysics::timeForDistance(ballVelRel, distanceToIntersection, theBallSpecification.friction);
    }
  }
}

void FieldBallProvider::checkIfBallIsInsideOwnPenaltyArea(FieldBall& fieldBall)
{
  fieldBall.isInsideOwnPenaltyArea = false;
  fieldBall.distanceToOwnPenaltyArea = -1.f;
  if(!fieldBall.ballWasSeen() && !theTeamBallModel.isValid)
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

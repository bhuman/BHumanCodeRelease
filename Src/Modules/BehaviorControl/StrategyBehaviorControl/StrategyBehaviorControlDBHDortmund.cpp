/**
 * @file StrategyBehaviorControlDBHDefender.cpp
 *
 * This file implements a module that determines the strategy of the team.
 *
 * @author Yannik Meinken
 */

#include "StrategyBehaviorControlDBHDortmund.h"

MAKE_MODULE(StrategyBehaviorControlDBHDortmund, behaviorControl);

void StrategyBehaviorControlDBHDortmund::update(SkillRequest& skillRequest)
{
  if(theGameState.playerState != GameState::active ||
     theGameState.isPenaltyShootout() || theGameState.isInitial() || theGameState.isReady() || theGameState.isFinished())
    skillRequest = SkillRequest::Builder::empty();
  else if(theGameState.isSet())
    skillRequest = SkillRequest::Builder::stand();
  else if(theGameState.isPlaying())
    skillRequest = behave();
}

SkillRequest StrategyBehaviorControlDBHDortmund::behave()
{
  switch(theGameState.playerNumber)
  {
    case 1:
      if(theFieldBall.ballWasSeen(300) &&
         theFieldBall.isRollingTowardsOwnGoal &&
         theFieldBall.positionRelative.squaredNorm() < sqr(3000.f))
      {
        const float interceptY = clip(theFieldBall.intersectionPositionWithOwnYAxis.y(), theFieldDimensions.yPosRightGoal + 150.f, theFieldDimensions.yPosLeftGoal - 150.f);
        return SkillRequest::Builder::walkTo(Pose2f(theFieldDimensions.xPosOwnGroundLine, interceptY));
      }
      if(theFieldBall.ballWasSeen(5000))
      {
        float positionY = mapToRange(theFieldBall.recentBallPositionOnField().y(), theFieldDimensions.yPosRightSideline, theFieldDimensions.yPosLeftSideline, theFieldDimensions.yPosRightGoal + 150.f, theFieldDimensions.yPosLeftGoal - 150.f);
        positionY /= (theFieldBall.positionRelative.norm() / 2000.f);
        return SkillRequest::Builder::walkTo(Pose2f(theFieldDimensions.xPosOwnGroundLine, positionY));
      }
      return SkillRequest::Builder::walkTo(Pose2f(theFieldDimensions.xPosOwnGroundLine, 0.f));
    case 3:
      if(theFieldBall.recentBallPositionOnField().x() > 0.f)
        return kickBall();

      return SkillRequest::Builder::stand();

    case 2:
      if(theFieldBall.recentBallPositionOnField().x() < 0)
        return kickBall();

      const Geometry::Line ballToGoal = Geometry::Line(theFieldBall.recentBallPositionOnField(), Vector2f(theFieldDimensions.xPosOwnGoal, 0) - theFieldBall.recentBallPositionOnField());
      const Geometry::Line line = Geometry::Line(Vector2f(-2000, 0), Vector2f(0, 1));

      Vector2f pos = Vector2f::Zero();
      if(Geometry::getIntersectionOfLines(ballToGoal, line, pos))
        return SkillRequest::Builder::walkTo(Pose2f((theFieldBall.recentBallPositionOnField() - pos).angle(), pos));
  }
  return SkillRequest::Builder::empty();
}

SkillRequest StrategyBehaviorControlDBHDortmund::kickBall()
{
  const Angle newAngleToBall = (theFieldBall.recentBallPositionOnField() - theRobotPose.translation).angle();
  // no angle adjustments that are minor or close to the ball
  float distanceToBall = theFieldBall.recentBallEndPositionRelative().norm();
  if(distanceToBall > 150.f && std::abs(Angle::normalize(angleToBall - newAngleToBall)) > 10_deg)
    angleToBall = newAngleToBall;
  return SkillRequest::Builder::dribbleTo(angleToBall);
}


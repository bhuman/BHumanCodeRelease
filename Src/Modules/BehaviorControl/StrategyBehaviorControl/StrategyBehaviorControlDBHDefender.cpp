/**
 * @file StrategyBehaviorControlDBHDefender.cpp
 *
 * This file implements a module that determines the strategy of the team.
 *
 * @author Yannik Meinken
 * @author Lars Bredereke
 */

#include "StrategyBehaviorControlDBHDefender.h"
#include "Debugging/DebugDrawings.h"
#include "Math/BHMath.h"

MAKE_MODULE(StrategyBehaviorControlDBHDefender, behaviorControl);

void StrategyBehaviorControlDBHDefender::update(SkillRequest& skillRequest)
{
  DECLARE_DEBUG_DRAWING("Strategy:path3", "drawingOnField");
  DECLARE_DEBUG_DRAWING("Strategy:mark2", "drawingOnField");
  if(theGameState.playerState != GameState::active ||
     theGameState.isPenaltyShootout() || theGameState.isInitial() || theGameState.isReady() || theGameState.isFinished())
    skillRequest = SkillRequest::Builder::empty();
  else if(theGameState.isSet())
    skillRequest = SkillRequest::Builder::stand();
  else if(theGameState.isPlaying())
    skillRequest = behave();
}

SkillRequest StrategyBehaviorControlDBHDefender::behave()
{
  const float distanceToBall = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();

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
    {
      if(theFieldBall.timeSinceBallWasSeen > ballSearchDelay && !theFieldBall.teammatesBallIsValid)
      {
        if(!ballFound && theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) < ballStillOnSpot)
          return SkillRequest::Builder::observe(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f));
        return searchBall();
      }
      timeBallSearchStarted = 0;
      ballFound = true;
      // position to intercept the first pass as long as the ball doesn't leave the opponents half
      if(distanceToBall < 1500.f && theFieldBall.recentBallPositionOnField().x() > 0.f)
        return kickBall();

      if(theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) < decideMirroredInterval)
        return SkillRequest::Builder::walkTo(Pose2f(theFieldDimensions.xPosOpponentPenaltyMark, 0));

      Teammate number2;
      for(Teammate t : theTeamData.teammates)
      {
        if(t.number == 2)
        {
          number2 = t;
          break;
        }
      }

      mirrored = number2.theRobotPose.translation.y() > 0;

      Geometry::Line line(theFieldBall.recentBallEndPositionOnField(), theFieldBall.recentBallEndPositionOnField() - Vector2f(0.f, (mirrored ? -1875.f : 1875.f)));
      Vector2f pointOnLine = Geometry::getOrthogonalProjectionOfPointOnLine(line, theRobotPose.translation);
      Vector2f dest = pointOnLine + (theFieldBall.recentBallEndPositionOnField() - pointOnLine) *  ballWeight / (Geometry::distance(theRobotPose.translation, pointOnLine));
      dest = dest.x() > theFieldBall.recentBallEndPositionOnField().x() ? theFieldBall.recentBallEndPositionOnField() : dest;
      CROSS("Strategy:path3", dest.x(), dest.y(), 30, 3, Drawings::solidPen, ColorRGBA::red);
      LINE("Strategy:path3", 0.f, mirrored ? -1875 : 1875.f, theFieldBall.recentBallEndPositionOnField().x(), theFieldBall.recentBallEndPositionOnField().y(), 3, Drawings::dashedPen, ColorRGBA::red);
      if(dest.x() > safetyDistanceToMid)
        return SkillRequest::Builder::walkTo(Pose2f((theFieldBall.recentBallEndPositionOnField() - dest).angle(), dest));

      dest.x() = safetyDistanceToMid;
      dest.y() = dest.y() * std::min(ballWeightOtherHalf / -theFieldBall.recentBallEndPositionOnField().x() + safetyDistanceToMid, 1.f);
      CROSS("Strategy:path3", dest.x(), dest.y(), 30, 3, Drawings::solidPen, ColorRGBA::red);
      return SkillRequest::Builder::walkTo(Pose2f((theFieldBall.recentBallEndPositionOnField() - dest).angle(), dest));
    }
    case 2:
      if((theFieldBall.timeSinceBallWasSeen > ballSearchDelay && !theFieldBall.teammatesBallIsValid) && theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) > ballStillOnSpot)
        return searchBall();

      timeBallSearchStarted = 0;
      if((distanceToBall < 1500.f || ballWasNearMarked) && theFieldBall.ballWasSeen(timeToTrustBall))
      {
        ballWasNearMarked = true;
        return kickBall();
      }

      if(theFieldBall.recentBallPositionOnField().x() < 0)
      {
        //if the ball is in the own half we have to go towards it
        return handleBallInOwnHalf();
      }

      Teammate number3;
      for(const Teammate& t : theTeamData.teammates)
      {
        if(t.number == 3)
        {
          number3 = t;
          break;
        }
      }
      Vector2f positionOfTeammate = number3.theRobotPose.translation;

      std::vector<Obstacle> obstaclesToMark;
      std::vector<Obstacle> obstacles;
      for(auto& o : theObstacleModel.obstacles)
      {
        Vector2f center = theRobotPose * o.center;
        if(o.type != Obstacle::teammate && center.x() < notFirstAttacker && (center - positionOfTeammate).norm() > sameObstacleThreshold)
          obstaclesToMark.push_back(o);
      }

      if(obstaclesToMark.empty())
      {
        //if the second attacker was already found he has walked outside of our field of view, so turn to find him again
        if(secondAttackerFound)
          return SkillRequest::Builder::walkTo(Pose2f(theRobotPose.rotation + (positionToMark.y() < theRobotPose.translation.y() ? -30_deg : 30_deg), theRobotPose.translation));
        mirrored = theRobotPose.translation.y() > 0;
        positionToMark = Vector2f(-600.f, mirrored ? 1700.f : -1700.f);
        return SkillRequest::Builder::mark(positionToMark);
      }

      secondAttackerFound = true;
      //mark the obstacle nearest to me
      Obstacle newObstacleToMark = *std::min_element(obstaclesToMark.begin(), obstaclesToMark.end(), [&](Obstacle o1, Obstacle o2) {return o1.center.squaredNorm() < o2.center.squaredNorm(); });
      CIRCLE("Strategy:mark2", positionToMark.x(), positionToMark.y(), sameObstacleThreshold + theFrameInfo.getTimeSince(timeWhenObstacleToMarkWasSeen) / 4, 20, Drawings::solidPen, ColorRGBA::violet, Drawings::noBrush, ColorRGBA::white);
      if(timeWhenObstacleToMarkWasSeen == 0 || (theRobotPose * newObstacleToMark.center - positionToMark).norm() < sameObstacleThreshold + theFrameInfo.getTimeSince(timeWhenObstacleToMarkWasSeen) / 4)
      {
        positionToMark = theRobotPose * newObstacleToMark.center;
        timeWhenObstacleToMarkWasSeen = newObstacleToMark.lastSeen;
      }

      if((positionToMark - theFieldBall.recentBallPositionOnField()).norm() < ballNearMarkedObstacle)
        ballWasNearMarked = true;

      if(theFrameInfo.getTimeSince(timeWhenObstacleToMarkWasSeen) > verifyOpponentToMarkTime)
      {
        if(theFrameInfo.getTimeSince(timeWhenObstacleToMarkWasSeen) > searchOpponentToMarkTime)
          return SkillRequest::Builder::walkTo(Pose2f(theRobotPose.rotation + (positionToMark.y() < theRobotPose.translation.y() ? -30_deg : 30_deg), theRobotPose.translation));
        return SkillRequest::Builder::observe(positionToMark);
      }

      CROSS("Strategy:mark2", positionToMark.x(), positionToMark.y(), 50, 10, Drawings::solidPen, ColorRGBA::violet);
      return SkillRequest::Builder::mark(positionToMark);
  }
  return SkillRequest::Builder::empty();
}

SkillRequest StrategyBehaviorControlDBHDefender::kickBall()
{
  const Angle newAngleToBall = (theFieldBall.recentBallPositionOnField() - theRobotPose.translation).angle();
  // no angle adjustments that are minor or close to the ball
  float distanceToBall = theFieldBall.recentBallEndPositionRelative().norm();
  if(distanceToBall > 150.f && std::abs(Angle::normalize(angleToBall - newAngleToBall)) > 10_deg)
    angleToBall = newAngleToBall;
  return SkillRequest::Builder::dribbleTo(angleToBall);
}

SkillRequest StrategyBehaviorControlDBHDefender::searchBall()
{
  if(timeBallSearchStarted == 0)
    timeBallSearchStarted = theFrameInfo.time;

  if(theFrameInfo.getTimeSince(timeBallSearchStarted) < ballSearchDuration)
    return SkillRequest::Builder::observe(theFieldBall.recentBallPositionOnField());

  switch(theGameState.playerNumber)
  {
    case 3:
      if((theFrameInfo.getTimeSince(timeBallSearchStarted) + ballSearchDuration) % (3 * ballSearchDuration) < ballSearchDuration)
        return SkillRequest::Builder::walkTo(Pose2f((Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f) - theRobotPose.translation).angle(), theRobotPose.translation));
      else if((theFrameInfo.getTimeSince(timeBallSearchStarted) + ballSearchDuration) % (3 * ballSearchDuration) < 2 * ballSearchDuration)
        return SkillRequest::Builder::walkTo(Pose2f((Vector2f(-300.f, theFieldBall.recentBallPositionOnField().y() > 0 ? 1700.f : -1700.f) - theRobotPose.translation).angle(), theRobotPose.translation));
      else
        return SkillRequest::Builder::walkTo(Pose2f((Vector2f(-300.f, theFieldBall.recentBallPositionOnField().y() < 0 ? 1700.f : -1700.f) - theRobotPose.translation).angle(), theRobotPose.translation));
      break;
    case 2:
      return SkillRequest::Builder::walkTo(Pose2f(theRobotPose.rotation + 20_deg, theRobotPose.translation));
      break;
    default:
      return SkillRequest::Builder::empty();
  }
}

SkillRequest StrategyBehaviorControlDBHDefender::handleBallInOwnHalf()
{
  if(theFieldBall.ballWasSeen(timeToTrustBall))
  {
    Vector2f pointToMark = positionToMark + (Vector2f(theFieldDimensions.xPosOwnGroundLine, 0) - positionToMark).normalized() * 500.f;

    Geometry::Line line(pointToMark, theFieldBall.recentBallPositionOnField() - pointToMark);
    Vector2f pointOnLine = Geometry::getOrthogonalProjectionOfPointOnLine(line, theRobotPose.translation);
    Vector2f dest = pointOnLine + (theFieldBall.recentBallEndPositionOnField() - pointOnLine) *  ballWeight / (Geometry::distance(theRobotPose.translation, pointOnLine));
    dest = Geometry::distance(dest, pointToMark) < Geometry::distance(theFieldBall.recentBallPositionOnField(), pointToMark) ? dest : theFieldBall.recentBallPositionOnField();
    LINE("Strategy:mark2", pointToMark.x(), pointToMark.y(), theFieldBall.recentBallEndPositionOnField().x(), theFieldBall.recentBallEndPositionOnField().y(), 3, Drawings::dashedPen, ColorRGBA::red);
    CROSS("Strategy:mark2", dest.x(), dest.y(), 30, 3, Drawings::solidPen, ColorRGBA::red);

    return SkillRequest::Builder::walkTo(Pose2f((theFieldBall.recentBallEndPositionOnField() - dest).angle(), dest));
  }
  return SkillRequest::Builder::observe(theFieldBall.recentBallPositionOnField());
}

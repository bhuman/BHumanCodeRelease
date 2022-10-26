/**
 * @file StrategyBehaviorControlDBHAttacker.cpp
 *
 * This file implements a module that determines the strategy of the team.
 *
 * @author Yannik Meinken
 * @author Sina Schreiber
 * @author Michelle Gusev
 */

#include "StrategyBehaviorControlDBHAttacker.h"

MAKE_MODULE(StrategyBehaviorControlDBHAttacker, behaviorControl);

void StrategyBehaviorControlDBHAttacker::update(SkillRequest& skillRequest)
{
  DECLARE_DEBUG_DRAWING("behavior:Attacker:position", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Attacker:heatmap", "drawingOnField");

  if(theGameState.playerState != GameState::active ||
     theGameState.isPenaltyShootout() || theGameState.isInitial() || theGameState.isReady() || theGameState.isFinished())
    skillRequest = SkillRequest::Builder::empty();
  else if(theGameState.isSet())
  {
    skillRequest = SkillRequest::Builder::stand();
    secondRobot = false;
    thirdRobot = false;
    shoot = true;
    pos = theRobotPose.translation;
  }
  else if(theGameState.isPlaying())
  {
    skillRequest = behave();
  }
}

SkillRequest StrategyBehaviorControlDBHAttacker::behave()
{
  const float distanceToBall = (theRobotPose.translation - theFieldBall.recentBallPositionOnField()).norm();

  Teammate number2;
  Teammate number3;

  int active = theGameState.playerNumber;
  float minDistanceToBall = theFieldBall.recentBallPositionRelative().squaredNorm();

  for(Teammate t : theTeamData.teammates)
  {
    if(t.number == 2)
    {
      number2 = t;
    }
    else if(t.number == 3)
    {
      number3 = t;
    }

    if(t.theBallModel.timeWhenLastSeen >= theFrameInfo.time - 2000 && minDistanceToBall > t.theBallModel.estimate.position.squaredNorm())
    {
      minDistanceToBall = t.theBallModel.estimate.position.squaredNorm();
      active = t.number;
    }
  }

  if(((number3.getEstimatedPosition(theFrameInfo.time)) - theFieldBall.recentBallPositionOnField()).norm() < maxDistanceToBall)
    thirdRobot = true;

  if(((number2.getEstimatedPosition(theFrameInfo.time)) - theFieldBall.recentBallPositionOnField()).norm() < maxDistanceToBall)
    secondRobot = true;

  switch(theGameState.playerNumber)
  {
    case 1: // behavior of robot nr. 1
    {
      //wait at the beginning
      if(theGameState.timeWhenStateStarted + timeWaiting >= theFrameInfo.time)
        return SkillRequest::Builder::stand();

      // the pass Rating for a pass to robot number 2 is higher than the rating to robot number 3, robot 1 will pass to robot 2
      if(active == theGameState.playerNumber && thePassEvaluation.getRating(number2.theRobotPose.translation) + (passTarget == 2) * passHysteresis > thePassEvaluation.getRating(number3.theRobotPose.translation) + (passTarget == 3) * passHysteresis)
      {
        passTarget = 2;
        return SkillRequest::Builder::passTo(2);
      }

      // the pass Rating for a pass to robot number 3 is higher than the rating to robot number 2, robot 1 will pass to robot 3
      else if(distanceToBall < maxDistanceToBall)
      {
        passTarget = 3;
        return SkillRequest::Builder::passTo(3);
      }

      //if the distance to the ball isnt in range walk to the center point
      return SkillRequest::Builder::walkTo(Pose2f(0.f, 0.f));

    // behavior of robot nr. 2 and 3
      case 2:
      case 3:
      {
        //drawRating();
        bool otherHadBall = theGameState.playerNumber == 2 ? thirdRobot : secondRobot;
        int otherNumber = theGameState.playerNumber == 2 ? 3 : 2;

        // if the ball was not near Robot 3, Robot 2 has to pass the ball to Robot 3
        if(active == theGameState.playerNumber && !otherHadBall)
        {
          return SkillRequest::Builder::passTo(otherNumber);
        }
        else if(active == theGameState.playerNumber && otherHadBall)
        {
          return smashOrPass(theGameState.playerNumber == 2 ? number3 : number2);
        }

        pos = theRobotPose.translation;

        gradientAscent(pos, 25);
        return SkillRequest::Builder::walkTo(Pose2f((theFieldBall.recentBallEndPositionOnField() - pos).angle(), pos));
        CROSS("behavior:Attacker:position", pos.x(), pos.y(), 100, 20, Drawings::solidPen, ColorRGBA::yellow);
      }
      default:
        return (SkillRequest::Builder::stand());
      }
      return (SkillRequest::Builder::stand());
  }
}

float StrategyBehaviorControlDBHAttacker::rating(const Vector2f pos) const
{
  const float passRating = thePassEvaluation.getRating(pos);
  const float goalRating = (secondRobot || thirdRobot) * theExpectedGoals.getRating(pos) + !(secondRobot || thirdRobot);

  const float ballDist = (theFieldBall.recentBallPositionOnField() - pos).norm();
  const float inRange = ballDist > 3500 ? 1 : std::exp(-0.5f * sqr(ballDist - 3500) / sqr(1000));

  const float inOtherHalf = 1 - (pos.x() < 0 ? 1 : std::exp(-0.5f * sqr(pos.x()) / sqr(500)));

  return passRating * goalRating * inRange * inOtherHalf;
}

void StrategyBehaviorControlDBHAttacker::gradientAscent(Vector2f& pos, int numIterations) const
{
  float d = delta;
  for(int i = 0; i < numIterations; i++)
  {
    const float value = rating(pos);

    const Vector2f dx = Vector2f(d, 0);
    const Vector2f dy = Vector2f(0, d);

    const float df_dx = rating(pos + dx) - value;
    const float df_dy = rating(pos + dy) - value;
    Vector2f direction = Vector2f(df_dx, df_dy);
    direction.normalize();

    if(direction == Vector2f(0, 0))
      d += delta;
    else
      d = delta;

    CROSS("behavior:Forward:position", pos.x() + dx.x(), pos.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);
    CROSS("behavior:Forward:position", pos.x(), pos.y() + dy.y(), 20, 5, Drawings::solidPen, ColorRGBA::red);

    const Vector2f oldP = pos;
    pos += direction * step;
    LINE("behavior:Forward:position", oldP.x(), oldP.y(), pos.x(), pos.y(), 50, Drawings::solidPen, ColorRGBA::yellow);
  }
}

void StrategyBehaviorControlDBHAttacker::drawRating()
{
  cellColors.clear();
  gridCornerUpper = upperFieldCorner;
  gridCornerLower = -upperFieldCorner;
  cellsNumber = ((gridCornerUpper - gridCornerLower).array() / Vector2f(100, 100).array()).cast<int>();
  cellsNumber += Vector2i(1, 1);
  totalCellsNumber = cellsNumber.x() * cellsNumber.y();
  cellColors.clear();
  cellColors.reserve(totalCellsNumber);
  for(float y = gridCornerLower.y(); y <= gridCornerUpper.y(); y += 100)
  {
    for(float x = gridCornerLower.x(); x <= gridCornerUpper.x(); x += 100)
    {
      // Get combined rating at each cell in the target area on the field
      const float value = rating(Vector2f(x, y));

      ColorRGBA color(0, 0, 0, static_cast<unsigned char>((1.f - value) * 200));
      cellColors.push_back(color);
    }
  }
  GRID_RGBA("behavior:Attacker:heatmap", 0, 0, 100, cellsNumber.x(), cellsNumber.y(), cellColors.data());
}

SkillRequest StrategyBehaviorControlDBHAttacker::smashOrPass(Teammate other)
{
  float shootRating = theExpectedGoals.getRating(theFieldBall.recentBallPositionOnField());
  float passRating = thePassEvaluation.getRating(other.getEstimatedPosition(theFrameInfo.time)) *
                     theExpectedGoals.getRating(other.getEstimatedPosition(theFrameInfo.time));
  shootRating += shoot ? 0.5f : -0.5f;
  if(shootRating > passRating)
  {
    shoot = true;
    return SkillRequest::Builder::shoot();
  }

  shoot = false;
  return SkillRequest::Builder::passTo(other.number);
}

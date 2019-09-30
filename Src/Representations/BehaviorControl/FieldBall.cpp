/**
 * @file FieldBall.cpp
 *
 * Declaration of a representation that contains additional information
 * about the ball that is required by the behavior.
 *
 * @author Tim Laue
 */

#include "FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Module/Blackboard.h"

bool FieldBall::ballWasSeen(int timeInterval) const
{
  return timeSinceBallWasSeen <= timeInterval;
}

Vector2f FieldBall::recentBallPositionOnField(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && timeSinceTeamBallWasValid < timeSinceBallWasSeen)
    return teamPositionOnField;
  else
    return positionOnField;
}

Vector2f FieldBall::recentBallPositionRelative(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && timeSinceTeamBallWasValid < timeSinceBallWasSeen)
    return teamPositionRelative;
  else
    return positionRelative;
}

Vector2f FieldBall::recentBallEndPositionOnField(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && timeSinceTeamBallWasValid < timeSinceBallWasSeen)
    return teamEndPositionOnField;
  else
    return endPositionOnField;
}

Vector2f FieldBall::recentBallEndPositionRelative(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && timeSinceTeamBallWasValid < timeSinceBallWasSeen)
    return teamEndPositionRelative;
  else
    return endPositionRelative;
}

void FieldBall::recentBallPositions(Vector2f& ballPositionOnField, Vector2f& ballPositionRelative, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && timeSinceTeamBallWasValid < timeSinceBallWasSeen)
  {
    ballPositionOnField = teamPositionOnField;
    ballPositionRelative = teamPositionRelative;
  }
  else
  {
    ballPositionOnField = positionOnField;
    ballPositionRelative = positionRelative;
  }
}

void FieldBall::recentBallEndPositions(Vector2f& ballEndPositionOnField, Vector2f& ballEndPositionRelative, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  if((timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
     && timeSinceTeamBallWasValid < timeSinceBallWasSeen)
  {
    ballEndPositionOnField = teamEndPositionOnField;
    ballEndPositionRelative = teamEndPositionRelative;
  }
  else
  {
    ballEndPositionOnField = endPositionOnField;
    ballEndPositionRelative = endPositionRelative;
  }
}

void FieldBall::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:FieldBall:global", "drawingOnField");
  DECLARE_DEBUG_DRAWING("representation:FieldBall:relative", "drawingOnField");
  if(isRollingTowardsOpponentGoal)
  {
    FieldDimensions* theFieldDimensions = nullptr;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      theFieldDimensions = static_cast<FieldDimensions*>(&(Blackboard::getInstance()["FieldDimensions"]));
      const float x1 = theFieldDimensions->xPosOpponentGoalPost;
      const float y1 = theFieldDimensions->yPosLeftGoal;
      const float x2 = theFieldDimensions->xPosOpponentGoal;
      const float y2 = theFieldDimensions->yPosRightGoal;
      FILLED_RECTANGLE("representation:FieldBall:global", x1, y1, x2, y2, 1, Drawings::solidPen, ColorRGBA(100, 255, 100), Drawings::solidBrush, ColorRGBA(100, 255, 100));
    }
  }
  if(isRollingTowardsOwnGoal)
  {
    FieldDimensions* theFieldDimensions = nullptr;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      theFieldDimensions = static_cast<FieldDimensions*>(&(Blackboard::getInstance()["FieldDimensions"]));
      const float x1 = theFieldDimensions->xPosOwnGoalPost;
      const float y1 = theFieldDimensions->yPosLeftGoal;
      const float x2 = theFieldDimensions->xPosOwnGoal;
      const float y2 = theFieldDimensions->yPosRightGoal;
      FILLED_RECTANGLE("representation:FieldBall:global", x1, y1, x2, y2, 1, Drawings::solidPen, ColorRGBA(255, 0, 0), Drawings::solidBrush, ColorRGBA(255, 0, 0));
    }
  }
  if(timeUntilIntersectsOwnYAxis != std::numeric_limits<float>::max() && intersectionPositionWithOwnYAxis != Vector2f::Zero())
  {
    CROSS("representation:FieldBall:relative", intersectionPositionWithOwnYAxis.x(), intersectionPositionWithOwnYAxis.y(), 150, 20, Drawings::solidPen, ColorRGBA::blue);
    LINE("representation:FieldBall:relative", 0.f, 0.f, intersectionPositionWithOwnYAxis.x() * 1.5f, intersectionPositionWithOwnYAxis.y() * 1.5f,
         10, Drawings::dashedPen, ColorRGBA::red);
    DRAWTEXT("representation:FieldBall:relative", intersectionPositionWithOwnYAxis.x() * 1.5f, intersectionPositionWithOwnYAxis.y() * 1.5f, 250, ColorRGBA::blue, timeUntilIntersectsOwnYAxis);
  }
}

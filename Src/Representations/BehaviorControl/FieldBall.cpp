/**
 * @file FieldBall.cpp
 *
 * Declaration of a representation that contains additional information
 * about the ball that is required by the behavior.
 *
 * @author Tim Laue
 */

#include "FieldBall.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/SensorData/RawInertialSensorData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Sensing/RobotModel.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Framework/Blackboard.h"
#include "Tools/Modeling/BallPhysics.h"

bool FieldBall::ballWasSeen(int timeInterval) const
{
  return timeSinceBallWasSeen <= timeInterval;
}

Vector2f FieldBall::recentBallPositionOnField(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamPositionOnField;
  else
    return positionOnField;
}

Vector2f FieldBall::recentBallPositionRelative(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamPositionRelative;
  else
    return positionRelative;
}

Vector2f FieldBall::recentBallEndPositionOnField(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamEndPositionOnField;
  else
    return endPositionOnField;
}

Vector2f FieldBall::recentBallEndPositionRelative(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamEndPositionRelative;
  else
    return endPositionRelative;
}

Vector2f FieldBall::recentBallPropagatedPositionOnField(const float t, const float friction, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  const bool useTeamEstimate = useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout);

  const Vector2f& ballPosition = useTeamEstimate ? teamPositionOnField : positionOnField;
  const Vector2f ballVelocity = useTeamEstimate ? teamVelocityOnField : velocityOnField;
  const Vector2f propagatedBallPosition = BallPhysics::propagateBallPosition(ballPosition, ballVelocity, t, friction);
  return propagatedBallPosition;
}

Vector2f FieldBall::recentBallPropagatedPositionRelative(const float t, const float friction, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  const bool useTeamEstimate = useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout);

  const Vector2f& ballPosition = useTeamEstimate ? teamPositionRelative : positionRelative;
  const Vector2f ballVelocity = useTeamEstimate ? teamVelocityRelative : velocityRelative;
  const Vector2f propagatedBallPosition = BallPhysics::propagateBallPosition(ballPosition, ballVelocity, t, friction);
  return propagatedBallPosition;
}

void FieldBall::recentBallPositions(Vector2f& ballPositionOnField, Vector2f& ballPositionRelative, const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout))
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
     && teammatesBallIsValid && teammatesBallNewerThanOwnBall)
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

bool FieldBall::isBallPositionConsistentWithGameState(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  if(useTeammatesBall(ballSeenTimeout, ballDisappearedTimeout))
    return teamBallPositionConsistentWithGameState;
  else
    return ballPositionConsistentWithGameState;
}

bool FieldBall::useTeammatesBall(const int ballSeenTimeout, const int ballDisappearedTimeout) const
{
  ASSERT(ballSeenTimeout >= 0);
  return (timeSinceBallWasSeen > ballSeenTimeout || timeSinceBallDisappeared > ballDisappearedTimeout)
         && teammatesBallIsValid && teammatesBallNewerThanOwnBall;
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

  DEBUG_DRAWING("representation:FieldBall", "drawingOnField")
  {
    if(timeSinceBallWasSeen < 10000
       && timeSinceBallDisappeared < 500)
    {
      const Vector2f& position(positionRelative);
      const Vector2f& velocity(velocityRelative);
      CIRCLE("representation:FieldBall",
             position.x(), position.y(), 45, 0, // pen width
             Drawings::solidPen, ColorRGBA::orange,
             Drawings::solidBrush, ColorRGBA::orange);
      ARROW("representation:FieldBall", position.x(), position.y(),
            position.x() + velocity.x(), position.y() + velocity.y(), 5, 1, ColorRGBA::orange);
    }
  }

  DEBUG_DRAWING3D("representation:FieldBall", "robot")
  {
    if(timeSinceBallWasSeen < 10000
       && timeSinceBallDisappeared < 500
       && Blackboard::getInstance().exists("BallSpecification")
       && Blackboard::getInstance().exists("RawInertialSensorData")
       && Blackboard::getInstance().exists("RobotModel"))
    {
      const BallSpecification& theBallSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
      const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
      const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
      const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
      TRANSLATE3D("representation:FieldBall", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:FieldBall", -orientation.x(), -orientation.y(), -orientation.z());
      RENDER_OPTIONS3D("representation:FieldBall", Drawings3D::disableOpacity | Drawings3D::disableDepth);
      const Vector2f position = recentBallPositionRelative();
      SPHERE3D("representation:FieldBall", position.x(), position.y(), theBallSpecification.radius,
               theBallSpecification.radius, ColorRGBA(255, 128, 0, 128));
    }
  }

  DEBUG_DRAWING3D("representation:FieldBall:sector", "robot")
  {
    if(timeSinceBallWasSeen < 10000
       && timeSinceBallDisappeared < 500
       && Blackboard::getInstance().exists("BallSpecification")
       && Blackboard::getInstance().exists("RawInertialSensorData")
       && Blackboard::getInstance().exists("RobotModel"))
    {
      const BallSpecification& theBallSpecification = static_cast<const BallSpecification&>(Blackboard::getInstance()["BallSpecification"]);
      const RawInertialSensorData& theRawInertialSensorData = static_cast<const RawInertialSensorData&>(Blackboard::getInstance()["RawInertialSensorData"]);
      const RobotModel& theRobotModel = static_cast<const RobotModel&>(Blackboard::getInstance()["RobotModel"]);
      const Vector3f orientation = theRawInertialSensorData.angle.cast<float>();
      TRANSLATE3D("representation:FieldBall:sector", 0.f, 0.f, std::min(theRobotModel.soleLeft.translation.z(), theRobotModel.soleRight.translation.z()));
      ROTATE3D("representation:FieldBall:sector", -orientation.x(), -orientation.y(), -orientation.z());
      const Vector2f position = recentBallPositionRelative();
      const Angle range = std::max(1_deg, static_cast<Angle>(std::asin(std::min(1.f, theBallSpecification.radius / position.norm()))));
      const Angle angle = position.angle();
      RING_SECTOR3D("representation:FieldBall:sector", Vector3f(0, 0, 4.f), angle - range, angle + range, 120.f, 180.f, ColorRGBA::orange);
    }
  }
}

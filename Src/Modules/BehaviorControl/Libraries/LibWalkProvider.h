/**
 * @file LibWalkProvider.h
 *
 * Recalculates a walktarget to avoid obstacles, fieldborders and penaltyareas
 *
 * @author Andreas Stolpmann
 */

#include "Framework/Module.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibWalkProvider,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(GameState),
  REQUIRES(IllegalAreas),
  REQUIRES(LibPosition),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  PROVIDES(LibWalk),
  LOADS_PARAMETERS(
  {,
    (int) ballTimeout,
    (float) obstacleAvoidanceDistance,
    (float) obstacleAvoidanceMinRadius,
    (float) obstacleAvoidanceMaxRadius,
    (float) goalPostAvoidanceDistance,
    (float) goalPostAvoidanceMinRadius,
    (float) goalPostAvoidanceMaxRadius,
    (float) ballAvoidanceDistance,
    (float) ballAvoidanceMinRadius,
    (float) ballAvoidanceMaxRadius,
    (float) penaltyAreaExpansion,
    (float) penaltyAreaAvoidanceDistance,
    (float) penaltyAreaAvoidanceMinRadius,
    (float) penaltyAreaAvoidanceMaxRadius,
    (float) fieldBorderExpansion,
    (float) fieldBorderAvoidanceDistance,
    (float) fieldBorderAvoidanceMinRadius,
    (float) fieldBorderAvoidanceMaxRadius,
  }),
});

class LibWalkProvider : public LibWalkProviderBase
{
  const Vector2f leftPenCorner = Vector2f(theFieldDimensions.xPosOwnPenaltyArea + penaltyAreaExpansion, theFieldDimensions.yPosLeftPenaltyArea + penaltyAreaExpansion);
  const Vector2f leftPenGoalLine = Vector2f(theFieldDimensions.xPosOwnGoalLine - penaltyAreaExpansion, theFieldDimensions.yPosLeftPenaltyArea + penaltyAreaExpansion);
  const Vector2f rightPenCorner = Vector2f(theFieldDimensions.xPosOwnPenaltyArea + penaltyAreaExpansion, theFieldDimensions.yPosRightPenaltyArea - penaltyAreaExpansion);
  const Vector2f rightPenGoalLine = Vector2f(theFieldDimensions.xPosOwnGoalLine - penaltyAreaExpansion, theFieldDimensions.yPosRightPenaltyArea - penaltyAreaExpansion);
  const Vector2f leftOpponentCorner = Vector2f(theFieldDimensions.xPosOpponentGoalLine + fieldBorderExpansion, theFieldDimensions.yPosLeftTouchline + fieldBorderExpansion);
  const Vector2f rightOpponentCorner = Vector2f(theFieldDimensions.xPosOpponentGoalLine + fieldBorderExpansion, theFieldDimensions.yPosRightTouchline - fieldBorderExpansion);
  const Vector2f leftOwnCorner = Vector2f(theFieldDimensions.xPosOwnGoalLine - fieldBorderExpansion, theFieldDimensions.yPosLeftTouchline + fieldBorderExpansion);
  const Vector2f rightOwnCorner = Vector2f(theFieldDimensions.xPosOwnGoalLine - fieldBorderExpansion, theFieldDimensions.yPosRightTouchline - fieldBorderExpansion);
  const Vector2f goalPosts[4] =
  {
    Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosLeftGoal),
    Vector2f(theFieldDimensions.xPosOpponentGoalPost, theFieldDimensions.yPosRightGoal),
    Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosLeftGoal),
    Vector2f(theFieldDimensions.xPosOwnGoalPost, theFieldDimensions.yPosRightGoal)
  };

  struct Obstacle
  {
    Angle angleLeft;
    Angle angleRight;
    Vector2f center;
    Vector2f left;
    Vector2f right;
    Vector2f outerLeft;
    Vector2f outerRight;
    bool active;
    Obstacle(const Angle angleLeft, const Angle angleRight, const Vector2f& center, const Vector2f& left, const Vector2f& right, const Vector2f& outerLeft, const Vector2f& outerRight)
      : angleLeft(angleLeft), angleRight(angleRight), center(center), left(left), right(right), outerLeft(outerLeft), outerRight(outerRight), active(true) {}
  };
  std::vector<Obstacle> obstacles;
  std::vector<Obstacle> obstaclesSortedLeft;
  std::vector<Obstacle> obstaclesSortedRight;

  Angle lastAvoidanceAngleOffset = 0.f;
  bool activeLastFrame = false;

  void calculateObstacles(const Pose2f& target, const Vector2f& targetOnField, const bool rough, const bool disableObstacleAvoidance, bool toBall);

  void addObstacle(const Vector2f& pos, const float radius, const float minAvoidanceRadius, const float maxAvoidanceRadius, const float avoidanceDistance);
  void addObstacle(const Vector2f& center, const Vector2f& left, const Vector2f& right, const float minAvoidanceRadius, const float maxAvoidanceRadius, const float avoidanceDistance);

  Angle getNextFreeAngle(const Angle baseAngle, const bool ccw);

  float getSqrDistanceToLine(const Vector2f& base, const Vector2f& dir, float length, const Vector2f& point, Vector2f& orthogonalProjection);

  MotionRequest::ObstacleAvoidance calcObstacleAvoidance(const Pose2f& target, bool rough, bool disableObstacleAvoidance, bool toBall);
  void update(LibWalk& libWalk) override;

  void deactivateRough();

  bool applyNotRough();

  void calculateAvoidanceAngleOffset(Angle& newAvoidanceAngleOffset, const Angle targetAngle, const Vector2f& targetOnField);
};

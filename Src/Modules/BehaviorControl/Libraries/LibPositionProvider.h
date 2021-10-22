/**
 * @file LibPositionProvider.h
 *
 * Contains helpful methods calculating positions.
 *
 * @author Martin Kroker
 * @author Andreas Stolpmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibPositionProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  PROVIDES(LibPosition),
  LOADS_PARAMETERS(
  {,
    (float) positionOffsetIfOccupied, /**< The radius around an obstacle that occupies my target position. */
    (Angle) deleteObstacleCircleRange, /**< A previous obstacle is only deleted if it is outside +- this angle. */
  }),
});

class LibPositionProvider : public LibPositionProviderBase
{
private:
  void update(LibPosition& libPosition) override;

  /**
   * @brief Determines whether the distance of the robot to the middle point
   * of the goal is greater than the specified value.
   * @param distance Distance to check.
   * @return Whether the robot is further away from its goal than the
   * specified distance.
   */
  bool distanceToOwnGoalGreaterThan(float distance) const;

  /**
   * Determines whether the given position is in the own goal area given a tolerance value.
   * Besides the tolerance, the field line width is already taken into account.
   * Thus a position on a line which belongs to the goal area will be considered
   * inside the area.
   * @param position Absolute position on the field
   * @param toleranceX tolerance on x axis.
   * @param toleranceY tolerance on y axis.
   * @return Whether the position is inside the own goal area.
   */
  bool isNearOwnGoalArea(const Vector2f& position, float toleranceX, float toleranceY) const;

  /**
   * Checks whether the given position is in the own goal area using no tolerance.
   * @param position Absolute position on the field
   * @return Whether the position is inside the own goal area.
   */
  bool isInOwnGoalArea(const Vector2f& position) const;

  /**
   * Determines whether the given position is in the own penalty area given a tolerance value.
   * Besides the tolerance, the field line width is already taken into account.
   * Thus a position on a line which belongs to the penalty area will be considered
   * inside the area.
   * @param position Absolute position on the field
   * @param toleranceX tolerance on x axis.
   * @param toleranceY tolerance on y axis.
   * @return Whether the position is inside the own penalty area.
   */
  bool isNearOwnPenaltyArea(const Vector2f& position, float toleranceX, float toleranceY) const;

  /**
   * Checks whether the given position is in the own penalty area using no tolerance.
   * @param position Absolute position on the field
   * @return Whether the position is inside the own penalty area.
   */
  bool isInOwnPenaltyArea(const Vector2f& position) const;

  /**
   * Determines whether the given position is in the opponents penalty area given a tolerance value.
   * Besides the tolerance, the field line width is already taken into account.
   * Thus a position on a line which belongs to the penalty area will be considered
   * inside the area.
   * @param position Absolute position on the field
   * @param toleranceX tolerance on x axis.
   * @param toleranceY tolerance on y axis.
   * @return Whether the position is inside the own penalty area.
   */
  bool isNearOpponentPenaltyArea(const Vector2f& position, float toleranceX, float toleranceY) const;

  /**
   * Checks whether the given position is in the opponents penalty area using no tolerance.
   * @param position Absolute position on the field
   * @return Whether the position is inside the own penalty area.
   */
  bool isInOpponentPenaltyArea(const Vector2f& position) const;

  /**
   * Checks whether the given position is outside + offset the own goal.
   * @param position Absolute position on the field
   * @return Whether the position far enough in the field
   */
  bool isOutSideGoalFrame(const Vector2f& position, const float offset) const;

  /**
   * Calculates the circle around an obstacle which occupies my target position.
   * @param position The potential target position on the field
   * @return The circle with the obstacle as center or radius 0 if none
   */
  Geometry::Circle getObstacleAtMyPositionCircle(const Vector2f& position);

  Geometry::Circle lastCircle;
};

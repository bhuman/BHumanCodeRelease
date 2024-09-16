/**
 * @file LibPositionProvider.h
 *
 * Contains helpful methods calculating positions.
 *
 * @author Martin Kroker
 * @author Andreas Stolpmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "Framework/Module.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibPositionProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  PROVIDES(LibPosition),
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
   * Calculates weather the goalkeeper is near a post or not.
   * @param position The realtaive position of the robot
   * @return A tuple of two booleans. The first one indicates if the robot is near the left post, the second one if the robot is near the right post.
   */
  std::tuple<bool, bool> isNearPost(const Pose2f& position) const;
};

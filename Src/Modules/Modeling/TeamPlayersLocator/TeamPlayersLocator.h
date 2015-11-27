/**
* @file TeamPlayersLocator.h
*
*
* @author Florian Maaß
*/

#pragma once

#include "Tools/Modeling/Obstacle.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/TeammateData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include <vector>

MODULE(TeamPlayersLocator,
{,
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  REQUIRES(BallModel),
  REQUIRES(TeammateData),
  REQUIRES(FieldDimensions),
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),
  REQUIRES(ObstacleModel),
  PROVIDES(TeamPlayersModel),
  LOADS_PARAMETERS(
  {,
    (float) squaredMahalanobisDistanceParameter,
    (float) selfDetectionOnlyRadius,
  }),
});

/**
 * @class TeamPlayersLocator
 * A combined world model
 */
class TeamPlayersLocator : public TeamPlayersLocatorBase
{
  /**
  * Provides the combined world model representation
  */
  const float squaredDistanceThreshold = sqr(Obstacle::getRobotDepth());
  void update(TeamPlayersModel& teamPlayersModel);

  Matrix2f rotateCovariance(const Matrix2f& matrix, const float angle);
  void merge(Obstacle& obstacle, std::vector<Obstacle>& obstacles) const;
  void removeAround(Obstacle& teammate, std::vector<Obstacle>& obstacles) const;
  Obstacle::Type setType(const Obstacle::Type one, const Obstacle::Type other) const;
  bool isInsideOwnDetectionArea(const Vector2f& position) const;
};

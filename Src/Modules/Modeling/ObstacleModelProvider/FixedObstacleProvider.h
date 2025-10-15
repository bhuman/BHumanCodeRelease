/**
 * @file FixedObstacleProvider.h
 *
 * Declaration of a module that places fixed obstacles into the obstacle model.
 *
 * @author Moritz Oppermann
 */
#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Framework/Module.h"

MODULE(FixedObstacleProvider,
{,
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  PROVIDES(ObstacleModel),
  LOADS_PARAMETERS(
  {,
    (std::vector<Vector2f>) fixedObstacleLocations,
    (Matrix2f) cov,
  }),
});

/*
 * @class FixedObstacleProvider
 *
 * Places fixed obstacles into the obstacle model.
 */
class FixedObstacleProvider : public FixedObstacleProviderBase
{
  /** The function is called when the representation provided needs to be updated. */
  void update(ObstacleModel& obstacleModel) override;
};

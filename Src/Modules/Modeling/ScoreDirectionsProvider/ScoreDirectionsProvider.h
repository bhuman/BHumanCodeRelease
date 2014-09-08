/**
 * @file ScoreDirectionsProvider.h
 * The file declares a module that determines sectors of directions in which the ball can be
 * kicked and hit the goal. This is based on the obstacle model. The problem is solved using
 * a sweep line method. All obstacles are described as angular sectors surrounding the ball.
 * The two edges of each sector, i.e. the direction in which an obstacle begins and the
 * direction in which it ends, are added to an array that is then sorted by the angles. The
 * goal posts are added as well. After that, the array is traversed while updating an
 * obstacle counter. Whenever a starting edge is encountered, the counter is increased, and
 * when an ending edge is found, the counter is lowered. Whenever the counter is zero, a
 * free sector was found.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ExpObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ScoreDirections.h"

MODULE(ScoreDirectionsProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(ExpObstacleModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  PROVIDES_WITH_MODIFY_AND_DRAW(ScoreDirections),
  DEFINES_PARAMETERS(
  {,
    (float)(1.0f) bonusRatio,
  }),
});

class ScoreDirectionsProvider : public ScoreDirectionsProviderBase
{
  /**
   * Method to update the provided representation.
   * @param scoreDirections Sectors of directions in which the ball
   *                        can be kicked and hit the goal.
   */
  void update(ScoreDirections& scoreDirections);
};

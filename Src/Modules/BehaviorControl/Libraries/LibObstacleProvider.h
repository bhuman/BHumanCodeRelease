/**
 * @file LibObstacleProvider.h
 *
 * Contains information about obstacles
 *
 * @author Andreas Stolpmann
 */

#include "Framework/Module.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/BehaviorControl/Libraries/LibObstacle.h"

MODULE(LibObstacleProvider,
{,
  REQUIRES(ObstacleModel),
  PROVIDES(LibObstacle),
  DEFINES_PARAMETERS(
  {,
    (float)(0.f) xMin,
    (float)(250.f) xMax,
    (float)(50.f) yMin,
    (float)(400.f) yMax,
  }),
});

class LibObstacleProvider : public LibObstacleProviderBase
{
private:
  void update(LibObstacle& libObstacle) override;
  bool isObstacleOnSide(const bool left) const;
  bool isObstacleInPath(const Vector2f& target, const float pathWidth, const float maxDistance) const;
};

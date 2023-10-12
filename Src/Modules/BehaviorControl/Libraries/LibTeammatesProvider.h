/**
 * @file LibTeammatesProvider.h
 *
 * Contains information about teammate positions
 *
 * @author Lukas Malte Monnerjahn
 */

#include "Framework/Module.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibTeammates.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibTeammatesProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(LibPosition),
  REQUIRES(RobotPose),
  REQUIRES(StrategyStatus),
  REQUIRES(TeamData),
  PROVIDES(LibTeammates),
  DEFINES_PARAMETERS(
  {,
    (float)(400.f) outsideDistanceThreshold,
    (float)(50.f) insideDistanceThreshold,
  }),
});

class LibTeammatesProvider : public LibTeammatesProviderBase
{
private:
  void update(LibTeammates& libTeammates) override;

  /**
   * Returns the amount of other teammates in the own goal area, not counting the goalkeeper.
   * Uses two different distance thresholds depending on this robot being already inside the goal area or not.
   * This prevents robots inside the goal area from running out because of nearby teammates who don't enter.
   * @return number of teammates in the own goal area excluding the goalkeeper
   */
  int nonKeeperTeammatesInOwnGoalArea() const;
};

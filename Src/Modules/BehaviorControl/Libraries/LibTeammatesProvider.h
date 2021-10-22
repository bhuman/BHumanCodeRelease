/**
 * @file LibTeammatesProvider.h
 *
 * Contains information about teammate positions
 *
 * @author Lukas Malte Monnerjahn
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibTeammates.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"

MODULE(LibTeammatesProvider,
{,
  REQUIRES(LibPosition),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  PROVIDES(LibTeammates),
  DEFINES_PARAMETERS({,
   (float) (400.f) outsideDistanceThreshold,
   (float) (50.f) insideDistanceThreshold,
   }),
});

class LibTeammatesProvider : public LibTeammatesProviderBase
{
private:
  void update(LibTeammates& libTeammates) override;

  /**
   * Returns the amount of other teammates in the own penalty area, not counting the goalkeeper.
   * Uses two different distance thresholds depending on this robot being already inside the penalty area or not.
   * This prevents robots inside the penalty area from running out because of nearby teammates who don't enter.
   * @return number of teammates in the own penalty area excluding the goalkeeper
   */
  int nonKeeperTeammatesInOwnPenaltyArea() const;

  /**
   * Returns the amount of other teammates in the opponents penalty area.
   * Uses two different distance thresholds depending on this robot being already inside the penalty area or not.
   * This prevents robots inside the penalty area from running out because of nearby teammates.
   * @return number of teammates in the opponents penalty area
   */
  int teammatesInOpponentPenaltyArea() const;
};
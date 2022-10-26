/**
 * @file TeamDataProvider.h
 *
 * This file declares a module that converts \c ReceivedTeamMessages to \c TeamData.
 * In the middle term, this module and \c TeamData should disappear.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Libraries/LibDemo.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/GameState.h"
#include "Framework/Module.h"

MODULE(TeamDataProvider,
{,
  USES(GameState),
  USES(LibDemo),
  REQUIRES(ReceivedTeamMessages),
  PROVIDES(TeamData),
});

class TeamDataProvider : public TeamDataProviderBase
{
  void update(TeamData& teamData) override;

  Teammate& getTeammate(TeamData& teamData, int number) const;

  void handleMessage(Teammate& teammate, const ReceivedTeamMessage& teamMessage) const;
};

/**
 * @file OpponentBreakingSupporterPositionProvider.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Math/BHMath.h"
#include "Tools/Module/Module.h"
#include "Representations/Modeling/OpponentBreakingSupporterPosition.h"
#include "Representations/Modeling/TeamPlayersModel.h"
#include "Representations/Configuration/FieldDimensions.h"

MODULE(OpponentBreakingSupporterPositionProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(TeamPlayersModel),

  PROVIDES(OpponentBreakingSupporterPosition),
  DEFINES_PARAMETERS(
  {,
    (float)(sqr(1000)) sqrMaxDistanceToSearchAreaCenter,
    (float)(0.5f) yFactorForOvalArea,
  }),
});

class OpponentBreakingSupporterPositionProvider : public OpponentBreakingSupporterPositionProviderBase
{
  void update(OpponentBreakingSupporterPosition& opponentOpponentBreakingSupporter);

  void updateWithNoInput(OpponentBreakingSupporterPosition& opponentOpponentBreakingSupporter);
  void updateWithInput(OpponentBreakingSupporterPosition& opponentOpponentBreakingSupporter, const std::vector<const Obstacle*> possibleOBs);

  std::vector<const Obstacle*> findPossibleOBSs();
};

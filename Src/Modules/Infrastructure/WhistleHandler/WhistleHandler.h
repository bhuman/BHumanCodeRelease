/**
 * @file WhistleHandler.h
 * Modifies the GameInfo to factor in if a whistle was
 * heard by the teammates to set the state to playing.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Module/Module.h"

MODULE(WhistleHandler,
{,
  USES(TeamBallModel),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(TeamData),
  REQUIRES(Whistle),
  PROVIDES(GameInfo),
  LOADS_PARAMETERS(
  {,
    (unsigned) maxTimeDifference,
    (bool) useBallPosition,
    (float) maxBallToMiddleDistance,
    (float) minAvgConfidence,
  }),
});

class WhistleHandler : public WhistleHandlerBase
{
private:
  std::vector<unsigned> penaltyTimes;

  unsigned timeOfLastSetState = 0;
  unsigned lastGameState = STATE_INITIAL;
  bool overrideGameState = false;

  bool checkWhistle() const;
  bool checkBall() const;

  bool checkForIllegalMotionPenalty();

  void update(GameInfo& gameInfo) override;
};

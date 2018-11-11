/**
 * @file WhistleHandler.h
 * Modifies the GameInfo to factor in if a whistle was
 * heard by the teammates to set the state to playing.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Module/Module.h"

MODULE(WhistleHandler,
{,
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(TeamData),
  REQUIRES(TeamBallModel),
  REQUIRES(Whistle),
  PROVIDES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(500) maxTimeDifference,
    (bool)(false) useBallPosition,
    (float)(300.f) maxBallToMiddleDistance,
    (float)(48.f) minAvgConfidence,
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

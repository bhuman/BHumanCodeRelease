/**
 * @file WhistleHandler.h
 * Modifies the GameInfo to factor in if a whistle was
 * heard by the teammates to set the state to playing.
 * @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
 */

#pragma once

#include "Representations/Communication/TeammateData.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Tools/Module/Module.h"

MODULE(WhistleHandler,
{,
  REQUIRES(CognitionStateChanges),
  REQUIRES(FrameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RawGameInfo),
  REQUIRES(TeammateData),
  REQUIRES(TeamBallModel),
  REQUIRES(Whistle),
  PROVIDES(GameInfo),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(500) maxTimeDifference,
    (bool)(false) useBallPosition,
    (float)(300.f) maxBallToMiddleDistance,
    (float)(50.f) minAvgConfidence,
  }),
});

class WhistleHandler : public WhistleHandlerBase
{
private:
  std::vector<unsigned> penaltyTimes;

  unsigned timeOfLastSetState = 0;
  bool overrideGameState = false;

  bool checkWhistle() const;
  bool checkBall() const;

  bool checkForIllegalMotionPenalty();

  void update(GameInfo& gameInfo);
};

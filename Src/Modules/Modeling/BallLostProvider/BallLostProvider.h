/**
 * @file BallLostProvider.h
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallLostModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/Odometer.h"
#include "Framework/Module.h"

MODULE(BallLostProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  REQUIRES(Odometer),
  PROVIDES(BallLostModel),
});

class BallLostProvider : public BallLostProviderBase
{
public:
  bool lastFrameDetected = false;
  bool blockUpdate = true;
  unsigned lastBallSeenTimestamp = 0;
  void update(BallLostModel& theBallLostModel) override;
};

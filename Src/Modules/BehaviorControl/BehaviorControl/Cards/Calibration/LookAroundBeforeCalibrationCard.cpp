/**
 * @file LookAroundBeforeCalibrationCard.cpp
 *
 * This file implements the behavior of a robot after it entered the calibration mode to make it look around.
 * Basically a copy of LookAroundAfterPenaltyCard.
 *
 * @author Arne Hasselbring
 * @author Lukas Plecher
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/Math/Random.h"

CARD(LookAroundBeforeCalibrationCard,
{,
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookLeftAndRight),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (int)(3000) duration, /**< This card will be active for at least this duration. */
  }),
});

class LookAroundBeforeCalibrationCard : public LookAroundBeforeCalibrationCardBase
{
  unsigned lookAroundStart;

  bool preconditions() const override
  {
    return theExtendedGameInfo.timeSinceLastCalibrationStarted == 0 ||
           theExtendedGameInfo.timeSincePlayingStarted == 0;
  }

  bool postconditions() const override
  {
    return theFrameInfo.getTimeSince(lookAroundStart) >= duration;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::lookAroundAfterPenalty);
    theLookLeftAndRightSkill(false);
    theStandSkill();
  }

  void reset() override
  {
    lookAroundStart = theFrameInfo.time;
  }
};

MAKE_CARD(LookAroundBeforeCalibrationCard);

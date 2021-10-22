/**
 * @file FallenCard.cpp
 *
 * This file specifies the behavior for a fallen robot.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(FallenCard,
{,
  CALLS(Annotation),
  CALLS(Activity),
  CALLS(GetUpEngine),
  CALLS(LookForward),
  REQUIRES(FallDownState),
  REQUIRES(MotionInfo),
});

class FallenCard : public FallenCardBase
{
  bool preconditions() const override
  {
    return theFallDownState.state == FallDownState::fallen ||
           (theFallDownState.state == FallDownState::squatting &&
            // sitDownKeeper is special because it automatically continues to a get up motion when being left.
            !theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDownKeeper) &&
            // Do not get up when we want to sit down.
            !theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDown));
  }

  bool postconditions() const override
  {
    return theFallDownState.state == FallDownState::upright;
  }

  void execute() override
  {
    theAnnotationSkill("Getting up.");
    theActivitySkill(BehaviorStatus::fallen);
    theLookForwardSkill();
    theGetUpEngineSkill();
  }
};

MAKE_CARD(FallenCard);

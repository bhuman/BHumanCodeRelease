/**
 * @file FallenCard.cpp
 *
 * This file specifies the behavior for a fallen robot.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(FallenCard,
{,
  CALLS(Annotation),
  CALLS(Activity),
  CALLS(GetUpEngine),
  CALLS(LookForward),
  REQUIRES(FallDownState),
});

class FallenCard : public FallenCardBase
{
  bool preconditions() const override
  {
    return theFallDownState.state == FallDownState::fallen ||
           theFallDownState.state == FallDownState::squatting;
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

/**
 * @file FinishedCard.cpp
 *
 * This file specifies the behavior for a robot in the Finished game state.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(FinishedCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  REQUIRES(GameInfo),
});

class FinishedCard : public FinishedCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_FINISHED;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_FINISHED;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::finished);
    theLookForwardSkill();
    theStandSkill();
  }
};

MAKE_CARD(FinishedCard);

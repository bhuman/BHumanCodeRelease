/**
 * @file SetCard.cpp
 *
 * This file specifies the behavior for a robot in the Set game state.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(SetCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(SpecialAction),
  REQUIRES(GameInfo),
});

class SetCard : public SetCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_SET;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_SET;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::set);
    theLookForwardSkill();
    theSpecialActionSkill(SpecialActionRequest::standHigh);
  }
};

MAKE_CARD(SetCard);

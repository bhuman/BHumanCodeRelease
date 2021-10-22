/**
 * @file ReplayWalkRequestCard.cpp
 *
 * This file implements a card that handles behavior to replay walk requests.
 *
 * @author Philip Reichenberg
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(ReplayWalkRequestCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(ReplayWalk),
});

class ReplayWalkRequestCard : public ReplayWalkRequestCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::unknown);
    theLookForwardSkill();
    theReplayWalkSkill();
  }
};

MAKE_CARD(ReplayWalkRequestCard);

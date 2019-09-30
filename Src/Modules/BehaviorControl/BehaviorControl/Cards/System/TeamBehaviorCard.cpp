/**
 * @file TeamBehaviorCard.cpp
 *
 * This file implements a card that handles the overall team behavior.
 *
 * @author Arne Hasselbring
 */

#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Tools/BehaviorControl/Framework/Card/TeamCard.h"

TEAM_CARD(TeamBehaviorCard,
{,
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<TeamCardRegistry>) teamBehaviors, /**< The deck from which to choose the team behavior. */
  }),
});

class TeamBehaviorCard : public TeamBehaviorCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    dealer.deal(teamBehaviors)->call();
  }

  void reset() override
  {
    dealer.reset();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_TEAM_CARD(TeamBehaviorCard);

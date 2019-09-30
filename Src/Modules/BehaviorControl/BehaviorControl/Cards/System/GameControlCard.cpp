/**
 * @file GameControlCard.cpp
 *
 * This file implements a card that handles the top level robot control in normal games.
 *
 * @author Arne Hasselbring
 */

#include "Representations/Communication/RobotInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(GameControlCard,
{,
  REQUIRES(RobotInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) deck, /**< The deck from which a card is played. */
  }),
});

class GameControlCard : public GameControlCardBase
{
  bool preconditions() const override
  {
    return theRobotInfo.penalty == PENALTY_NONE;
  }

  bool postconditions() const override
  {
    return theRobotInfo.penalty != PENALTY_NONE;
  }

  void execute() override
  {
    dealer.deal(deck)->call();
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameControlCard);

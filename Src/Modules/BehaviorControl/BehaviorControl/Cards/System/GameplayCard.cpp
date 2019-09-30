/**
 * @file GameplayCard.cpp
 *
 * This file implements a card that represents the behavior of a robot in states in which it is not forced to do nothing.
 *
 * @author Arne Hasselbring
 */

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(GameplayCard,
{,
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ownKickoff,
    (DeckOfCards<CardRegistry>) opponentKickoff,
    (DeckOfCards<CardRegistry>) ownFreeKick,
    (DeckOfCards<CardRegistry>) opponentFreeKick,
    (DeckOfCards<CardRegistry>) normalPlay,
  }),
});

class GameplayCard : public GameplayCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_READY || theGameInfo.state == STATE_PLAYING;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_READY && theGameInfo.state != STATE_PLAYING;
  }

  void execute() override
  {
    if(theGameInfo.state == STATE_READY)
    {
      if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
      {
        dealer.deal(ownKickoff)->call();
        setState("ownKickoff");
      }
      else
      {
        dealer.deal(opponentKickoff)->call();
        setState("opponentKickoff");
      }
    }
    else
    {
      ASSERT(theGameInfo.state == STATE_PLAYING);
      if(theGameInfo.setPlay != SET_PLAY_NONE)
      {
        if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
        {
          dealer.deal(ownFreeKick)->call();
          setState("ownFreeKick");
        }
        else
        {
          dealer.deal(opponentFreeKick)->call();
          setState("opponentFreeKick");
        }
      }
      else
      {
        dealer.deal(normalPlay)->call();
        setState("normalPlay");
      }
    }
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameplayCard);

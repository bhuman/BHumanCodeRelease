/**
 * @file GameplayCard.cpp
 *
 * This file implements a card that represents the behavior of a robot in states in which it is not forced to do nothing.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(GameplayCard,
{,
  CALLS(ArmContact),
  CALLS(ArmObstacleAvoidance),
  CALLS(CountDownHalfTime),
  CALLS(TeamCountdown),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ownKickoff,
    (DeckOfCards<CardRegistry>) opponentKickoff,
    (DeckOfCards<CardRegistry>) ownFreeKick,
    (DeckOfCards<CardRegistry>) opponentFreeKick,
    (DeckOfCards<CardRegistry>) normalPlay,
    (DeckOfCards<CardRegistry>) ownPenaltyKick,
    (DeckOfCards<CardRegistry>) opponentPenaltyKick,
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
    theArmContactSkill();
    if(!theTeamBehaviorStatus.role.isGoalkeeper())
      theArmObstacleAvoidanceSkill();
    if(theGameInfo.state == STATE_READY)
    {
      if(theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
      {
        if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
        {
          dealer.deal(ownPenaltyKick)->call();
          setState("ownPenaltyKick");
        }
        else
        {
          dealer.deal(opponentPenaltyKick)->call();
          setState("opponentPenaltyKick");
        }
      }
      else
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
    }
    else
    {
      ASSERT(theGameInfo.state == STATE_PLAYING);
      #ifdef TARGET_ROBOT
        const int stateTime = theFrameInfo.time - _context.stateStart;
      #endif

      if(theGameInfo.setPlay != SET_PLAY_NONE)
      {
        if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
        {
          if(theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
          {
            dealer.deal(ownPenaltyKick)->call();
            setState("ownPenaltyKick");
          }
          else
          {
            dealer.deal(ownFreeKick)->call();
            setState("ownFreeKick");
          }
        }
        else
        {
          if(theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
          {
            dealer.deal(opponentPenaltyKick)->call();
            setState("opponentPenaltyKick");
          }
          else
          {
            dealer.deal(opponentFreeKick)->call();
            setState("opponentFreeKick");
          }
        }
      }
      else
      {
        dealer.deal(normalPlay)->call();
        setState("normalPlay");

        #ifdef TARGET_ROBOT
          if(stateTime <= 11000)
            theTeamCountdownSkill(stateTime);
        #endif
      }

      #ifdef TARGET_ROBOT
        if(theOpponentTeamInfo.teamNumber > 0 && stateTime > 11000)
          theCountDownHalfTimeSkill();
      #endif
    }
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(GameplayCard);

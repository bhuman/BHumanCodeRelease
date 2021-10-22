/**
 * @file SetCard.cpp
 *
 * This file specifies the behavior for a robot in the Set game state.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(SetCard,
{,
  CALLS(Activity),
  CALLS(LookActive),
  CALLS(LookAtPoint),
  CALLS(LookForward),
  CALLS(Stand),
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
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
    if(theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      theLookForwardSkill();
    else if((theGameInfo.setPlay == SET_PLAY_PENALTY_KICK && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber) ? theTeamBehaviorStatus.role.isGoalkeeper() : theTeamBehaviorStatus.role.playsTheBall())
    {
      const Vector2f targetOnField = theGameInfo.setPlay == SET_PLAY_PENALTY_KICK ?
                                     Vector2f(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark, 0.f) :
                                     Vector2f::Zero();
      theLookAtPointSkill((Vector3f() << theRobotPose.inversePose * targetOnField, theBallSpecification.radius).finished());
    }
    else
      theLookActiveSkill(/* withBall: */ false, /* ignoreBall: */ true);
    theStandSkill(/* high: */ true);
  }
};

MAKE_CARD(SetCard);

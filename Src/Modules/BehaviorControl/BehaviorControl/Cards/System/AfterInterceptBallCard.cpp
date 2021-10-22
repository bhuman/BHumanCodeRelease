/**
 * @file AfterInterceptBallCard.cpp
 *
 * This file implements a card which takes over control when the robot
 * executes a diving motion which would normally trigger the FallenCard.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(AfterInterceptBallCard,
{,
  CALLS(Activity),
  CALLS(AfterInterceptBall),
  REQUIRES(FallDownState),
  REQUIRES(GameInfo),
  USES(MotionRequest),
});

class AfterInterceptBallCard : public AfterInterceptBallCardBase
{
  bool preconditions() const override
  {
    return theMotionRequest.motion == MotionRequest::keyframeMotion &&
           (theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectFromSitting ||
            theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStand ||
            theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStandDefender ||
            theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::keeperJumpLeft);
  }

  bool postconditions() const override
  {
    return // The following line is currently needed due to the FallEngine. It assures that the next action that will be done is getting up.
           theFallDownState.state != FallDownState::falling &&
           theAfterInterceptBallSkill.isDone();
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::afterInterceptBall);
    theAfterInterceptBallSkill(/* allowGetUp: */ theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT || theGameInfo.state != STATE_PLAYING);
  }
};

MAKE_CARD(AfterInterceptBallCard);

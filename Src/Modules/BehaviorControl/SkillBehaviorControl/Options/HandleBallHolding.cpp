#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleBallHolding)
{
  initial_state(noBallHolding)
  {
    transition
    {
      if(theBallHoldingState.ballHolding)
        goto freeBall;
    }
  }

  state(freeBall)
  {
    transition
    {
      if((!theBallHoldingState.ballHolding && state_time > 2000) || theFieldBall.timeSinceBallWasSeen == 0)
        goto noBallHolding;
    }
    action
    {
      if(state_time == 0)
        Say({.text = "Ball Holding",
             .force = true});
      const Vector2f goal(theFieldDimensions.xPosOpponentGoal, 0.f);
      FreeBallHolding({.targetDirection = (theRobotPose.inverse() * goal).angle()});
      LookAtAngles({.pan = 0.f,
                    .tilt = 23_deg});
    }
  }
}

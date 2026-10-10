/**
 * @file DemoSoccer.cpp
 *
 * This file implements the DemoSoccer skill.
 *
 * @author Philip Reichenberg et al.
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoSoccer)
{
  initial_state(searching)
  {
    transition
    {
      // Only play the ball if we saw it some time before far away or very recently.
      if((theFieldBall.ballWasSeen(4000) && theBallModel.estimate.position.squaredNorm() > sqr(500.f)) ||
              theFieldBall.ballWasSeen(2000))
        goto playing;
    }
    action
    {
      DemoSearchForBall();
    }
  }

  state(playing)
  {
    transition
    {
      // Do not play the ball if we have not seen it for a while or we have lost it while it was close.
      if(!theFieldBall.ballWasSeen(4000) ||
              (!theFieldBall.ballWasSeen(2000) && theBallModel.estimate.position.squaredNorm() <= sqr(500.f)))
        goto searching;
    }
    action
    {
      DemoGoToBallAndKick();
    }
  }
}

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleAnyPlaceDemo)
{
  common_transition
  {
    switch(theLibDemo.demoGameState)
    {
      case LibDemo::soccer:
      {
        // Only play the ball if we saw it some time before far away or very recent close
        if((theFieldBall.ballWasSeen(4000) && theBallModel.estimate.position.squaredNorm() > sqr(500.f)) ||
           theFieldBall.ballWasSeen(2000))
          goto playing;
        else
          goto searching;
      }
      case LibDemo::talking:
        goto talking;
      case LibDemo::waving:
      {
        if(theMotionInfo.isMotion(MotionPhase::stand))
          goto waving;
        else
          goto walkToWaving;
      }
      case LibDemo::posing:
        goto posing;
      default:
        FAIL("Unknown demo game state.");
    }
  }

  initial_state(initial)
  {
  }

  state(playing)
  {
    action
    {
      DemoGoToBallAndKick();
    }
  }

  state(searching)
  {
    action
    {
      DemoSearchForBall();
    }
  }

  state(talking)
  {
    action
    {
      DemoTalk();
    }
  }

  state(walkToWaving)
  {
    transition
    {
      if(state_time > 3000)
        goto waving;
    }
    action
    {
      if(state_time < 2000)
      {
        WalkAtRelativeSpeed({.speed = {0.f, 0.01f, 0.f}});
      }
      else
        Stand();
      LookForward();
    }
  }

  state(waving)
  {
    action
    {
      DemoWave();
    }
  }

  state(posing)
  {
    action
    {
      theOptionalImageRequest.sendImage = true;
      DemoPose();
    }
  }
}

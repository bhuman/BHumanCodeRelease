option(HandleAnyPlaceDemo)
{
  common_transition
  {
    switch(theLibDemo.demoGameState)
    {
      case LibDemo::normal:
        goto playing;
      case LibDemo::talking:
        goto talking;
      case LibDemo::waving:
        goto waving;
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
      if(theFieldBall.ballWasSeen(4000))
        theDemoGoToBallAndKickSkill();
      else
        theDemoSearchForBallSkill();
    }
  }

  state(talking)
  {
    action
    {
      theDemoTalkSkill();
    }
  }

  state(waving)
  {
    action
    {
      theDemoWaveSkill();
    }
  }
}

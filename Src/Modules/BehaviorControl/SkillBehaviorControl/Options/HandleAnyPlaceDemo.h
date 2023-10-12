option(HandleAnyPlaceDemo)
{
  common_transition
  {
    switch(theLibDemo.demoGameState)
    {
      case LibDemo::soccer:
        if(theFieldBall.ballWasSeen(4000))
          goto playing;
        else
          goto searching;
      case LibDemo::talking:
        goto talking;
      case LibDemo::waving:
        goto waving;
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
      theDemoGoToBallAndKickSkill();
    }
  }

  state(searching)
  {
    action
    {
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

  state(posing)
  {
    action
    {
      theOptionalImageRequest.sendImage = true;
      theDemoPoseSkill();
    }
  }
}

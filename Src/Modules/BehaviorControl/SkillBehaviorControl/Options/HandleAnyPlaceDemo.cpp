#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleAnyPlaceDemo)
{
  initial_state(initial)
  {
    transition
    {
      // States can only be switched while penalized, so it's okay that this is not a common transition.
      switch(theLibDemo.demoGameState)
      {
        case LibDemo::soccer:
          goto soccer;
        case LibDemo::waving:
          goto waving;
        case LibDemo::talking:
          goto talking;
        case LibDemo::posing:
          goto posing;
      }
    }
  }

  state(soccer)
  {
    action
    {
      DemoSoccer();
    }
  }

  state(waving)
  {
    action
    {
      DemoWave();
    }
  }

  state(talking)
  {
    action
    {
      DemoTalk();
    }
  }

  state(posing)
  {
    action
    {
      theRefereeDetectionRequest.detectReferee = true;
      DemoPose();
    }
  }
}

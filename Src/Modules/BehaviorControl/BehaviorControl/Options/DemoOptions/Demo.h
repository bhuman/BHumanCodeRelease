/**
 * @file Demo.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

option(Demo)
{
  common_transition
  {
    switch(theLibDemo.demoGameState)
    {
      case LibDemo::waving:
        goto waving;
      case LibDemo::normal:
        goto normal;
      default:
        FAIL("Unknown demo game state.");
    }
  }

  initial_state(normal)
  {
    action
    {
      Striker();
      theHeadControlMode = HeadControl::lookForward;
    }
  }

  state(waving)
  {
    action
    {
      Activity(BehaviorStatus::waving);
      theHeadControlMode = HeadControl::lookForward;
      Waving();
    }
  }
}

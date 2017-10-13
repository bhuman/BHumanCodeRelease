option(Demo)
{
  common_transition
  {
    switch(libDemo.demoGameState)
      {
      case LibDemo::wavingPlinking:
        goto waving;
      case LibDemo::normal:
        goto normal;
      default:
        ASSERT(false);
        break;
    }
  }

  initial_state(normal)
  {
    action
    {
      Goaler();
      LookForward();
    }
  }

  state(waving)
  {
    action
    {
      LookForward();
      Waving();
    }
  }
}

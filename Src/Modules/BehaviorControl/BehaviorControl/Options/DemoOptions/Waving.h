option(Waving)
{
  initial_state(initial)
  {
    transition
    {
      if(theLibDemo.changeArmToWave)
        theLibDemo.setArmToWave(Arms::Arm((theLibDemo.armToWave + 1) % Arms::numOfArms));
      goto playing;
    }
    action
    {
      SpecialAction(SpecialActionRequest::standHighLookUp);
    }
  }

  state(playing)
  {
    transition
    {
      if(state_time > 5000)
        goto waiting;
    }
    action
    {
      SpecialAction(SpecialActionRequest::standHighLookUp);
    }
  }

  state(waiting)
  {
    transition
    {
      if(state_time > 5000)
        goto initial;
    }
    action
    {
      SpecialAction(SpecialActionRequest::standHighLookUp);
    }
  }
}

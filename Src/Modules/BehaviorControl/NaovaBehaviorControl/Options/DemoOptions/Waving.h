option(Waving)
{
  initial_state(initial)
  {
    transition
    {
      libDemo.lastWaveStart = theFrameInfo.time;
      libDemo.armToWing = Arms::Arm((libDemo.armToWing + 1) % Arms::numOfArms);
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
      if(theFrameInfo.getTimeSince(libDemo.lastWaveStart) > 5000)
        goto waiting;
    }
      action
    {
      //if(libDemo.armToWing == Arms::left)
      //  WaveLeftArm();
      //else
      //  WaveRightArm();

      SpecialAction(SpecialActionRequest::standHighLookUp);
    }
  }

  state(waiting)
  {
    transition
    {
      if(theFrameInfo.getTimeSince(libDemo.lastWaveStart) > 10000)
        goto initial;
    }
      action
    {
      SpecialAction(SpecialActionRequest::standHighLookUp);
    }
  }
}

option(Arm, ArmMotionRequest::Arm arm, ArmMotionRequest::ArmMotionId motion, bool fast = false,
  bool autoReverse = false, int autoReverseTime = 0)
{
  initial_state(setRequest)
  {
    transition
    {
      if (theArmMotionEngineOutput.arms[arm].move && theArmMotionEngineOutput.arms[arm].motion == motion)
        goto requestIsExecuted;
    }
    action
    {
      theArmMotionRequest.motion[arm] = motion;
      theArmMotionRequest.fast[arm] = fast;
      theArmMotionRequest.autoReverse[arm] = autoReverse;
      theArmMotionRequest.autoReverseTime[arm] = autoReverseTime;
    }
  }



  target_state(requestIsExecuted)
  {
    transition
    {
      if (!theArmMotionEngineOutput.arms[arm].move || theArmMotionEngineOutput.arms[arm].motion != motion)
        goto setRequest;
    }
    action
    {
      theArmMotionRequest.motion[arm] = motion;
      theArmMotionRequest.fast[arm] = fast;
      theArmMotionRequest.autoReverse[arm] = autoReverse;
      theArmMotionRequest.autoReverseTime[arm] = autoReverseTime;
    }
  }
}
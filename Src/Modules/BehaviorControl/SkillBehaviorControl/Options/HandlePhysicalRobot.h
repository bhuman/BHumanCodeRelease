option(HandlePhysicalRobot)
{
  common_transition
  {
    if(!theCameraStatus.ok)
      goto cameraBroken;
#ifndef NDEBUG
    else if(theRobotHealth.batteryLevel <= 1)
      goto lowBattery;
#endif
    else
      goto working;
  }

  initial_state(working)
  {}

  state(cameraBroken)
  {
    action
    {
      theLookAtAnglesSkill({.pan = JointAngles::off,
                            .tilt = JointAngles::off});
      thePlayDeadSkill();
    }
  }

#ifndef NDEBUG
  state(lowBattery)
  {
    action
    {
      theLookAtAnglesSkill({.pan = JointAngles::off,
                            .tilt = JointAngles::off});
      thePlayDeadSkill();
    }
  }
#endif
}

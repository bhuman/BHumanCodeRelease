#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePhysicalRobot)
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
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }

#ifndef NDEBUG
  state(lowBattery)
  {
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }
#endif
}

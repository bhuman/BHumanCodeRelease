#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleStoppedState)
{
  common_transition
  {
    if(!theEmergencyButtonState.buttonPressed && !theGameState.stopped)
      goto notStopped;
  }

  initial_state(notStopped)
  {
    transition
    {
      if(theMotionInfo.isMotion(MotionPhase::playDead)
         || (theEmergencyButtonState.buttonPressed
             && (theMotionInfo.isMotion(MotionPhase::fall)
                 || theMotionInfo.isMotion(MotionPhase::getUp))))
        goto fall;
      else
        goto stand;
    }
  }

  state(stand)
  {
    action
    {
      LookForward();
      Stand();
    }
  }

  state(fall)
  {
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }
}

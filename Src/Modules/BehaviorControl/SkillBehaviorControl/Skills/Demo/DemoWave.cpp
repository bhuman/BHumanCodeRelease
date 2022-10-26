/**
 * @file DemoWave.cpp
 *
 * This file implements an implementation of the DemoWave skill.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(DemoWaveImpl,
{,
  IMPLEMENTS(DemoWave),
  CALLS(LookAtAngles),
  CALLS(LookForward),
  CALLS(Special),
  CALLS(Stand),
  REQUIRES(EnhancedKeyStates),
});

class DemoWaveImpl : public DemoWaveImplBase
{
  option(DemoWave)
  {
    initial_state(initial)
    {
      transition
      {
        if(theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 1)
          goto setup;
        else if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1 || state_time > 10000)
          goto startWaving;
      }
      action
      {
        theLookAtAnglesSkill({.pan = 0.f,
                              .tilt = 0_deg});
        theStandSkill({.high = true});
      }
    }

    state(startWaving)
    {
      transition
      {
        if(theSpecialSkill.isDone())
          goto waving;
      }
      action
      {
        theLookForwardSkill();
        theSpecialSkill({.request = MotionRequest::Special::demoBannerWave});
      }
    }

    state(waving)
    {
      transition
      {
        if(theStandSkill.isDone())
          goto waiting;
      }
      action
      {
        theLookAtAnglesSkill({.pan = 0.f,
                              .tilt = 0_deg});
        theStandSkill({.high = true});
      }
    }

    state(waiting)
    {
      transition
      {
        if(state_time > 21000)
          goto initial;
        else if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1)
          goto startWaving;
      }
      action
      {
        theLookAtAnglesSkill({.pan = 0.f,
                              .tilt = 0_deg});
        theStandSkill({.high = true});
      }
    }

    state(setup)
    {
      transition
      {
        if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1)
          goto waving;
      }
      action
      {
        theLookForwardSkill();
        theSpecialSkill({.request = MotionRequest::Special::demoBannerWaveInitial});
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(DemoWaveImpl);

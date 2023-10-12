/**
 * @file DemoWave.cpp
 *
 * This file implements an implementation of the DemoWave skill.
 *
 * @author Arne Hasselbring, Sina Schreiber
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(DemoWaveImpl,
{,
  IMPLEMENTS(DemoWave),
  CALLS(LookAtAngles),
  CALLS(LookForward),
  CALLS(KeyFrameSingleArm),
  CALLS(KeyFrameArms),
  CALLS(Stand),
  REQUIRES(ArmMotionInfo),
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
          goto wavingRightArm;
      }
      action
      {
        theLookAtAnglesSkill({.pan = 0.f,
                              .tilt = 0_deg});
        theStandSkill({.high = true});
      }
    }
    state(wavingRightArm)
    {
      transition
      {
        if(theArmMotionInfo.armMotion[Arms::left] == ArmMotionRequest::keyFrame
           && theArmMotionInfo.armKeyFrameRequest.arms[Arms::left].motion == ArmKeyFrameRequest::ArmKeyFrameId::wavingInitial
           && theArmMotionInfo.isFinished[Arms::left]
           && theArmMotionInfo.armMotion[Arms::right] == ArmMotionRequest::keyFrame
           && theArmMotionInfo.armKeyFrameRequest.arms[Arms::right].motion == ArmKeyFrameRequest::ArmKeyFrameId::waving1
           && theArmMotionInfo.isFinished[Arms::right]
          )
        {
          goto wavingLeftArm;
        }
      }
      action
      {
        theLookForwardSkill();
        theStandSkill({.high = true });
        theKeyFrameSingleArmSkill({.motion = ArmKeyFrameRequest::wavingInitial, .arm = Arms::left});
        theKeyFrameSingleArmSkill({.motion = ArmKeyFrameRequest::waving1, .arm = Arms::right});
      }
    }
    state(wavingLeftArm)
    {
      transition
      {
        if(theArmMotionInfo.armMotion[Arms::right] == ArmMotionRequest::keyFrame
           && theArmMotionInfo.armKeyFrameRequest.arms[Arms::right].motion == ArmKeyFrameRequest::ArmKeyFrameId::wavingInitial
           && theArmMotionInfo.isFinished[Arms::right]
           && theArmMotionInfo.armMotion[Arms::left] == ArmMotionRequest::keyFrame
           && theArmMotionInfo.armKeyFrameRequest.arms[Arms::left].motion == ArmKeyFrameRequest::ArmKeyFrameId::waving1
           && theArmMotionInfo.isFinished[Arms::left]
          )
        {
          goto waiting;
        }
      }
        action
      {
        theLookForwardSkill();
        theStandSkill({.high = true });
        theKeyFrameSingleArmSkill({.motion = ArmKeyFrameRequest::wavingInitial, .arm = Arms::right});
        theKeyFrameSingleArmSkill({.motion = ArmKeyFrameRequest::waving1, .arm = Arms::left});
     }
    }

    state(initializeWaving)
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
        if(state_time > 1000)
          goto initial;
        else if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1)
          goto wavingRightArm;
      }
      action
      {
        if(state_time == 0)
        {
          theKeyFrameArmsSkill({.motion = ArmKeyFrameRequest::ArmKeyFrameId::useDefault});
        }
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
          goto initializeWaving;
      }
      action
      {
        theLookForwardSkill();
        theStandSkill({.high = true });
        theKeyFrameArmsSkill({.motion = ArmKeyFrameRequest::wavingInitial});
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(DemoWaveImpl);

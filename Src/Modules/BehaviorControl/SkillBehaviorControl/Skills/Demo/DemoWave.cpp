/**
 * @file DemoWave.cpp
 *
 * This file implements the DemoWave skill.
 *
 * @author Arne Hasselbring, Sina Schreiber
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoWave)
{
  initial_state(initial)
  {
    transition
    {
      if(theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 1 && state_time > 1000)
        goto setup;
      else if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1 || state_time > 10000)
        goto wavingRightArm;
    }
    action
    {
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
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
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
      KeyFrameLeftArm({.motion = ArmKeyFrameRequest::wavingInitial});
      KeyFrameRightArm({.motion = ArmKeyFrameRequest::waving1});
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
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
      KeyFrameRightArm({.motion = ArmKeyFrameRequest::wavingInitial});
      KeyFrameLeftArm({.motion = ArmKeyFrameRequest::waving1});
    }
  }

  state(initializeWaving)
  {
    transition
    {
      if(action_done)
        goto waiting;
    }
    action
    {
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
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
        KeyFrameArms({.motion = ArmKeyFrameRequest::ArmKeyFrameId::useDefault});
      }
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
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
      LookAtAngles({.pan = 0.f,
                    .tilt = 0_deg});
      Stand({.high = true});
      KeyFrameArms({.motion = ArmKeyFrameRequest::wavingInitial});
    }
  }
}

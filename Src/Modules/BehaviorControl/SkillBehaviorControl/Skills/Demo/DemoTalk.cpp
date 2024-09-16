/**
 * @file DemoTalk.cpp
 *
 * This file implements the DemoTalk skill.
 *
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoTalk)
{
  action
  {
    LookAtAngles({.pan = 0.f,
                  .tilt = 25_deg});
    Stand({.high = true});
    DemoTalkWaitForKey();
  }
}

option((SkillBehaviorControl) DemoTalkWaitForKey)
{
  initial_state(waiting)
  {
    transition
    {
      if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1)
        goto talk1;
      else if(theEnhancedKeyStates.hitStreak[KeyStates::headMiddle] == 1)
        goto talk2;
      else if(theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 1)
        goto talk3;
    }
  }

  state(talk1) {transition {if(action_done) goto waiting;} action {DemoWaitAndPlay({.soundName = "demo1.wav"});}}
  state(talk2) {transition {if(action_done) goto waiting;} action {DemoWaitAndPlay({.soundName = "demo2.wav"});}}
  state(talk3) {transition {if(action_done) goto waiting;} action {DemoWaitAndPlay({.soundName = "demo3.wav"});}}
}

option((SkillBehaviorControl) DemoWaitAndPlay,
       args((const std::string&) soundName),
       defs((int)(1000) delayBeforeTalking))
{
  initial_state(waiting)
  {
    transition
    {
      if(state_time > delayBeforeTalking)
        goto playBack;
    }
  }

  state(playBack)
  {
    transition
    {
      goto done;
    }
    action
    {
      SystemCall::playSound(soundName.c_str());
    }
  }

  target_state(done) {}
}

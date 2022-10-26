/**
 * @file DemoTalk.cpp
 *
 * This file implements an implementation of the DemoTalk skill.
 *
 * @author Arne Hasselbring
 */

#include "Platform/SystemCall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"

SKILL_IMPLEMENTATION(DemoTalkImpl,
{,
  IMPLEMENTS(DemoTalk),
  CALLS(LookAtAngles),
  CALLS(Stand),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(FrameInfo),
  DEFINES_PARAMETERS(
  {,
    (int)(1000) delayBeforeTalking, /**< This amount of time is waited after pressing the head button until the sound is actually played.*/
  }),
});

class DemoTalkImpl : public DemoTalkImplBase
{
  void execute(const DemoTalk&) override
  {
    theLookAtAnglesSkill({.pan = 0.f,
                          .tilt = 25_deg});
    theStandSkill({.high = true});
    if(idle)
    {
      if(theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 1)
        delaySound("demo1.wav");
      else if(theEnhancedKeyStates.hitStreak[KeyStates::headMiddle] == 1)
        delaySound("demo2.wav");
      else if(theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 1)
        delaySound("demo3.wav");
    }
    else if(theFrameInfo.getTimeSince(timeWhenStartedWaiting) > delayBeforeTalking)
    {
      SystemCall::playSound(soundName);
      idle = true;
    }
  }

  void reset(const DemoTalk&) override
  {
    idle = true;
  }

  void delaySound(const char* name)
  {
    idle = false;
    timeWhenStartedWaiting = theFrameInfo.time;
    soundName = name;
  }

  bool idle; /**< Whether the robot should say nothing. */
  unsigned timeWhenStartedWaiting; /**< The timestamp when a talking action was requested. */
  const char* soundName; /**< The sound that should be played in the future. */
};

MAKE_SKILL_IMPLEMENTATION(DemoTalkImpl);

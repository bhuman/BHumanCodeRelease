/**
 * @file BehaviorState.cpp
 *
 * This file implements the BehaviorState skill.
 *
 * @author Philip Reichenberg
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) BehaviorStateTransitionFailed)
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStateInformation.transitionFailed = true;
      theLibCheck.inc(LibCheck::behaviorState);
    }
  }
}

option((SkillBehaviorControl) BehaviorStateRemoteControlActive)
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStateInformation.remoteControlActive = true;
      theLibCheck.inc(LibCheck::behaviorState);
    }
  }
}

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleReplayWalk)
{
  initial_state(inactive)
  {
    transition
    {
      goto replay;
    }
  }

  state(replay)
  {
    action
    {
      ReplayWalk();
      LookForward();
    }
  }
}

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl)HandlePlayingOptions)
{
  // The default state is "playing".
  initial_state(initial)
  {
    action
    {
      if(!select_option(playingOptions))   //@playingOptions
        executeRequest();
    }
  }
}

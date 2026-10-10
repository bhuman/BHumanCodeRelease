#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePlayingOptions,
       load((std::vector<cabsl::Cabsl<SkillBehaviorControl>::OptionInfos::Option>) options))
{
  // The default state is "playing".
  initial_state(initial)
  {
    action
    {
      if(!select_option(options))
      {
        executeRequest();
        handleWalkSpeedAtFieldBorder();
      }
    }
  }
}

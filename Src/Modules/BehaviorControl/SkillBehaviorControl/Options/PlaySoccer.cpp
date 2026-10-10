/**
 * @file PlaySoccer.h
 *
 * This file defines the root option of the soccer behavior.
 *
 * @author Thomas Röfer
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) PlaySoccer,
       load((std::vector<cabsl::Cabsl<SkillBehaviorControl>::OptionInfos::Option>) options))
{
  initial_state(selectOption)
  {
    action
    {
      select_option(options);
    }
  }
}

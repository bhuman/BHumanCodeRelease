#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleAnyPlaceDemo)
{
  initial_state(initial)
  {
    action
    {
      DemoSoccer();
    }
  }
}

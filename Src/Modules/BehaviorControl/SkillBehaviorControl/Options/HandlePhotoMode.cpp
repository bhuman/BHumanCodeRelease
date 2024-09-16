#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePhotoMode)
{
  initial_state(notInPhotoMode)
  {
    transition
    {
      if(true) // TODO: Update condition
        goto photoMode;
    }
  }

  state(photoMode)
  {
    action
    {
      PhotoMode(); // TODO: Create setPhotoMode skill
      LookForward();
    }
  }
}

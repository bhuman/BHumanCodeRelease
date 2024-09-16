#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePlayerState)
{
  common_transition
  {
    switch(theGameState.playerState)
    {
      case GameState::unstiff:
        goto unstiff;
      case GameState::calibration:
        goto calibration;
      case GameState::active:
        if(theExtendedGameState.wasPenalized())
        {
          ANNOTATION("Behavior", "Unpenalized");
          SystemCall::say("Not penalized");
        }
        goto playing;
      default:
        if(!theExtendedGameState.wasPenalized())
        {
          ANNOTATION("Behavior", "Penalized " << TypeRegistry::getEnumName(theGameState.playerState));
          SystemCall::say("Penalized");
        }
        goto penalized;
    }
    FAIL("Unknown player state.");
  }

  initial_state(playing)
  {}

  state(unstiff)
  {
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }

  state(calibration)
  {
    action
    {
      CalibrationControl();
    }
  }

  state(penalized)
  {
    action
    {
      LookForward();
      Stand({.high = true});
    }
  }
}

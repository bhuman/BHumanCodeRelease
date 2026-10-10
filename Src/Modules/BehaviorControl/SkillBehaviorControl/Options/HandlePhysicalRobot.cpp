#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandlePhysicalRobot,
       defs((unsigned int)(2000) pressDuration, /**< Button press duration to switch from a safe state into a less safe one. */
            (Angle)(15_deg) maxTorsoForWalk, /**< Max Torso orientation to switch to active state. */
            (Angle)(15_deg) maxGyroForWalk, /**< Max gyro to switch to active state. */
            (Angle)(5_deg) maxGyroForFall, /**< Max gyro to switch to fall state. */
            (Angle)(60_deg) minFallAngle)) /**< Min Torso orientation to switch to fall state. */
{
  common_transition
  {
    if(theSystemSensorData.emergencyMode || theGameState.playerState == GameState::unstiff)
      goto inactive;
    else if(SystemCall::getMode() == SystemCall::simulatedRobot && theGameState.playerState != GameState::unstiff && theExtendedGameState.playerStateLastFrame == GameState::unstiff)
      goto active;
    else if(theMotionInfo.executedPhase != MotionPhase::prepare &&
            ((theEnhancedKeyStates.pressed[KeyStates::stand] && theMotionInfo.executedPhase != MotionPhase::playDead && theMotionInfo.executedPhase != MotionPhase::fall)
             || (theGameState.playerState != GameState::unstiff && theExtendedGameState.playerStateLastFrame == GameState::unstiff)))
      goto prepare;
  }

  // The remaining behavior is executed.
  initial_state(active)
  {}

  // The robot is inactive. This state can only be reached on Booster
  // robots and can only be left by pressing "stand".
  state(inactive)
  {
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});

      // Robot is still in damping, but the other buttons are pressed -> let LED blink fast
      if(theKeyStates.pressed[KeyStates::walk])
        BehaviorStateTransitionFailed();

      PlayDead();
    }
  }

  // Prepare for switching to the active state. This state can only be
  // reached by pressing the "stand" button, which only exists on Booster
  // robots.
  state(prepare)
  {
    transition
    {
      if(theEnhancedKeyStates.pressedDuration[KeyStates::walk] > pressDuration
         && theMotionInfo.executedPhase == MotionPhase::prepare
         && std::abs(theInertialData.angle.x()) < maxTorsoForWalk
         && std::abs(theInertialData.angle.y()) < maxTorsoForWalk
         && std::abs(theInertialData.gyro.x()) < maxGyroForWalk
         && std::abs(theInertialData.gyro.y()) < maxGyroForWalk
         && std::abs(theInertialData.gyro.z()) < maxGyroForWalk)
        goto active;

      if(theEnhancedKeyStates.pressedDuration[KeyStates::stand] > pressDuration
         && theMotionInfo.executedPhase == MotionPhase::prepare
         && theEnhancedKeyStates.hitStreak[KeyStates::walk] == 3
         && (std::abs(theInertialData.angle.x()) > minFallAngle || std::abs(theInertialData.angle.y()) > minFallAngle)
         && std::abs(theInertialData.gyro.x()) < maxGyroForFall
         && std::abs(theInertialData.gyro.y()) < maxGyroForFall
         && std::abs(theInertialData.gyro.z()) < maxGyroForFall)
        goto fall;
    }
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});

      // Conditions are not met -> let LED blink fast
      if(theKeyStates.pressed[KeyStates::walk]
         && theMotionInfo.executedPhase == MotionPhase::prepare
         && (std::abs(theInertialData.angle.x()) > maxTorsoForWalk
             || std::abs(theInertialData.angle.y()) > maxTorsoForWalk
             || std::abs(theInertialData.gyro.x()) > maxGyroForWalk
             || std::abs(theInertialData.gyro.y()) > maxGyroForWalk
             || std::abs(theInertialData.gyro.z()) > maxGyroForWalk))
        BehaviorStateTransitionFailed();

      Prepare();
    }
  }

  state(fall)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::fall)
        goto active;
    }
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      FallRequest();
    }
  }
}

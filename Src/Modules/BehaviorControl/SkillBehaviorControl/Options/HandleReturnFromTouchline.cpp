/**
 * @file HandleReturnFromTouchline.cpp
 *
 * This file defines an option that handles the return of a robot from the
 * touchlines, either because a robot is unpenalized, or because it is the
 * beginning of a half or the end of a timeout and all robots walk in from the
 * touchlines. The robot stays where it is and performs a left-right sweep with
 * its head in order to become certain of its pose and its surroundings before
 * entering the field. The scan is skipped if the time left in the ready state
 * is below a certain threshold (e.g. because the initial-to-ready gesture has
 * been missed, or the robot entered late for other reasons).
 *
 * @author Yannik Meinken
 * @author Arne Hasselbring
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleReturnFromTouchline,
       defs((Angle)(70_deg) maxAngle, /**< The outermost pan angle of the head sweep. */
            (Angle)(50_deg) maxSpeed, /**< The maximum speed (per s) of the head during the sweep. */
            (Angle)(20_deg) tilt, /**< The constant tilt angle of the head during the sweep. */
            (int)(20000) minTimeLeftInReady)) /**< The minimum time (in ms) that must be left in the ready state to enter the option. */
{
  initial_state(initialized)
  {
    transition
    {
      if((theExtendedGameState.wasPenalized() &&
          theGameState.isPlaying() &&
          theExtendedGameState.playerStateLastFrame != GameState::penalizedIllegalMotionInSet) ||
         (theExtendedGameState.wasInitial() &&
          theGameState.isReady() &&
          theGameState.kickOffSetupFromTouchlines &&
          -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds) > minTimeLeftInReady &&
          !theInitialToReady.isPawn))
        goto lookAround;
    }
  }

  state(lookAround)
  {
    transition
    {
      if(state_time > 4 * maxAngle.toDegrees() / maxSpeed.toDegrees() * 1000)
        goto initialized;
    }

    action
    {
      Stand();
      LookLeftAndRight({.startLeft = true,
                        .maxPan = maxAngle,
                        .tilt = tilt,
                        .speed = maxSpeed});
    }
  }
}

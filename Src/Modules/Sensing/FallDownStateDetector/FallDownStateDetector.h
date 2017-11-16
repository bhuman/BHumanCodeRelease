/**
 * @file FallDownStateDetector.h
 *
 * This file declares a module that determines whether the robot is upright,
 * staggering, falling, or on the ground. Its core idea is to use the
 * orientation of the support foot relative to the ground rather than the
 * orientation of the torso.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Cabsl.h"
#include "Tools/Module/Module.h"

MODULE(FallDownStateDetector,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(RobotModel),
  PROVIDES(FallDownState),
  DEFINES_PARAMETERS(
  {,
    (Vector2a)(15_deg, 15_deg) maxFootOrientationToKeepUpright,
    (Vector2a)(25_deg, 25_deg) maxFootOrientationToKeepStaggering,
    (Vector2a)(55_deg, 55_deg) minTorsoOrientationToDetermineDirection,
    (Angle)(10_deg) maxGyroToRegainStableState,
    (int)(100) minTimeToRegainStableState,
    (float)(200.f) minTorsoAboveGroundToRegainUpright,
    (float)(150.f) maxTorsoAboveGroundToKeepUpright,
    (bool)(true) playSounds,
    (int)(0) minTimeWithoutGroundContactToAssumePickup,
    (int)(3000) minTimeUntilStaggeringIsStable,
  }),
});

class FallDownStateDetector : public FallDownStateDetectorBase, public Cabsl<FallDownStateDetector>
{
  FallDownState* theFallDownState; /**< Pointer to the fall down state updated.*/
  Vector2a orientation; /**< The orientation of the support foot relative to the ground (in radians). */
  FallDownState::Direction direction; /**< The fall direction. Always computed, even if not falling. */
  float torsoAboveGround; /**< The distance of the torso above the ground (in mm). */

  bool footUpright; /**< Is the foot's orientation within upright limits? */
  bool footStaggering; /**< Is the foot's orientation within staggering limits? */
  bool stable; /**< Torso has very limited rotational motion. */
  bool toUpright; /**< Are all conditions for a return to upright from staggering met? */
  bool toStableUpright; /**< Are all conditions for a return to upright from a fall met? */
  bool toAlmostStableUpright;
  bool toSquatting; /**< Are all conditions for a switch to squatting from a stable state are met? */
  bool toStableSquatting; /**< Are all conditions for a return to squatting from a fall met? */
  bool useTorsoOrientation; /**< Torso orientation is big enough to be used. */

  void update(FallDownState& fallDownState);

  /**
   * Sets the output of this module.
   * @param state The fall down state.
   * @param direction The direction of the fall.
   * @param odometryRotationOffset The odometry offset created by the most recent state transition.
   */
  void setState(FallDownState::State state, FallDownState::Direction direction = FallDownState::none,
                Angle odometryRotationOffset = 0_deg);

  /**
   * Sets the output state and adds an odometry offset if the direction changed.
   * @param state The fall down state.
   */
  void setStateWithPossibleDirectionChange(FallDownState::State state);

  /**
   * Plays a sound of a given name if playback is activated.
   * @param file The name of the sound file. ".wav" will be appended automatically.
   */
  void playSound(const char* file) const;

  /**
   * The root option sets the fall down states for undefined, upright, and
   * staggering. A fall is handled by the suboption Fall.
   */
  option(Root)
  {
    initial_state(pickedUp)
    {
      transition
      {
        if(toStableUpright && theGroundContactState.contact)
          goto pickedUpToUpright;
        else if(toStableSquatting && theGroundContactState.contact)
          goto pickedUpToSquatting;
      }
      action
      {
        setState(FallDownState::pickedUp);
      }
    }

    state(pickedUpToUpright)
    {
      transition
      {
        if(!toStableUpright || !theGroundContactState.contact)
          goto pickedUp;
        else if(state_time >= minTimeToRegainStableState)
        {
          playSound("upright");
          goto upright;
        }
      }
      action
      {
        setState(FallDownState::pickedUp);
      }
    }

    state(pickedUpToSquatting)
    {
      transition
      {
        if(!toStableSquatting || !theGroundContactState.contact)
          goto pickedUp;
        else if(state_time >= minTimeToRegainStableState)
        {
          playSound("squatting");
          goto squatting;
        }
      }
      action
      {
        setState(FallDownState::pickedUp);
      }
    }

    state(upright)
    {
      transition
      {
        if(!footStaggering)
        {
          playSound("falling");
          goto fall;
        }
        else if(!footUpright)
          goto staggering;
        else if(!theGroundContactState.contact)
          goto uprightToPickedUp;
        else if(toSquatting)
        {
          playSound("squatting");
          goto squatting;
        }
      }
      action
      {
        setState(FallDownState::upright);
      }
    }

    state(uprightToPickedUp)
    {
      transition
      {
        if(!footStaggering)
        {
          playSound("falling");
          goto fall;
        }
        else if(!footUpright)
          goto staggering;
        else if(theGroundContactState.contact)
          goto upright;
        else if(state_time >= minTimeWithoutGroundContactToAssumePickup)
        {
          playSound("pickedUp");
          goto pickedUp;
        }
      }
      action
      {
        setState(FallDownState::upright);
      }
    }

    state(staggering)
    {
      transition
      {
        if(!footStaggering)
        {
          playSound("falling");
          goto fall;
        }
        else if(footUpright)
          goto staggeringToUpright;
      }
      action
      {
        setState(FallDownState::staggering);
      }
    }

    state(staggeringToUpright)
    {
      transition
      {
        if(!footStaggering)
        {
          playSound("falling");
          goto fall;
        }
        else if(!footUpright)
          goto staggering;
        else if(state_time >= minTimeToRegainStableState)
          goto upright;
      }
      action
      {
        setState(FallDownState::staggering);
      }
    }

    state(fall)
    {
      transition
      {
        if(toStableUpright)
        {
          playSound("upright");
          goto upright;
        }
        else if(toStableSquatting)
        {
          playSound("squatting");
          goto squatting;
        }
        else if(toAlmostStableUpright)
          goto fallToUpright;
      }
      action
      {
        Fall();
      }
    }

    state(fallToUpright)
    {
      transition
      {
        if(toStableUpright)
        {
          playSound("upright");
          goto upright;
        }
        else if(toStableSquatting)
        {
          playSound("squatting");
          goto squatting;
        }
        else if(!toAlmostStableUpright)
          goto fall;
        else if(state_time >= minTimeUntilStaggeringIsStable)
        {
          playSound("upright");
          goto upright;
        }
      }
      action
      {
        Fall();
      }
    }

    state(squatting)
    {
      transition
      {
        if(!footStaggering)
        {
          playSound("falling");
          goto fall;
        }
        else if(toStableUpright)
        {
          playSound("upright");
          goto upright;
        }
        else if(!theGroundContactState.contact)
          goto squattingToPickedUp;
      }
      action
      {
        setState(FallDownState::squatting, direction);
      }
    }

    state(squattingToPickedUp)
    {
      transition
      {
        if(!footStaggering)
        {
          playSound("falling");
          goto fall;
        }
        else if(theGroundContactState.contact)
          goto squatting;
        else if(state_time >= minTimeWithoutGroundContactToAssumePickup)
        {
          playSound("pickedUp");
          goto pickedUp;
        }
      }
      action
      {
        setState(FallDownState::squatting);
      }
    }
  }

  /**
   * This option handles a fall and sets the fall down states falling and onGround.
   * If the torso's orientation is big enough, it also updates the odometry to
   * handle situations where the robot is on its side and then rolls on its front
   * or back.
   */
  option(Fall)
  {
    initial_state(fallingWithoutOdometryUpdate)
    {
      transition
      {
        if(useTorsoOrientation)
          goto fallingWithOdometryUpdate;
      }
      action
      {
        setState(FallDownState::falling, direction);
      }
    }

    state(fallingWithOdometryUpdate)
    {
      transition
      {
        if(!useTorsoOrientation)
          goto fallingWithoutOdometryUpdate;
        else if(stable)
          goto fallingToFallen;
      }
      action
      {
        setStateWithPossibleDirectionChange(FallDownState::falling);
      }
    }

    state(fallingToFallen)
    {
      transition
      {
        if(!useTorsoOrientation)
          goto fallingWithoutOdometryUpdate;
        else if(!stable)
          goto fallingWithOdometryUpdate;
        else if(state_time >= minTimeToRegainStableState)
        {
          playSound("fallen");
          goto fallen;
        }
      }
      action
      {
        setStateWithPossibleDirectionChange(FallDownState::falling);
      }
    }

    state(fallen)
    {
      action
      {
        setStateWithPossibleDirectionChange(FallDownState::fallen);
      }
    }
  }
};

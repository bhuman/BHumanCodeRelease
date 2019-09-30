/**
 * Class to represent an Arm Motion. It consists of an array of
 * angle+stiffness settings to command consecutively to an arm. This class
 * is used by the ArmKeyFrameEngine to read definitions of arm motions from
 * a configuration file.
 * @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Tools/Math/Angle.h"

STREAMABLE(ArmKeyFrameMotion,
{
  /**
   * Represents one single state that an arm should reach. Consists of
   * target angles to be reached, stiffness data and amount of motion steps
   * defining the duration to reach the target angles.
   */
  STREAMABLE(ArmAngles,
  {,
    (std::vector<Angle>)({0.f, 0.f, 0.f, 0.f, 0.f, 0.f}) angles, /**< Array of size 6, containing target angles for shoulder+elbow joint */
    (std::vector<int>)({0, 0, 0, 0, 0, 0}) stiffness, /**< Array of size 6, containing stiffness data to set while targetting the above angles */
    (int)(4) steps, /**< Duration in motion frame for reaching the target angles from the current position */
  });

  /**
   * Creates a new arm motion representing the reverse motion of this one. That is
   * it will contain the same array of target states but in reverse order with the
   * provided state as target state. Inteneded use is to reverse any arm motion to
   * reach back to the default position of the arm, that's why the default position
   * must be passed to this method. It will be appended as target state to the
   * created motion.
   * @param defaultPos Angle definition of the arm's default position.
   * @return A new ArmKeyFrameMotion targetting the arm#s default position.
   */
  ArmKeyFrameMotion reverse(ArmAngles defaultPos)
  {
    ASSERT(id != ArmKeyFrameRequest::reverse); // reverse motion not reversible

    ArmKeyFrameMotion result;
    result.id = ArmKeyFrameRequest::reverse;

    if(id == ArmKeyFrameRequest::useDefault)
    {
      result.states = std::vector<ArmAngles>(states);
      return result;
    }

    result.states = std::vector<ArmAngles>();

    // add states in reverse order and skip the last state
    for(std::vector<ArmAngles>::reverse_iterator it = ++states.rbegin(); it != states.rend(); ++it)
    {
      result.states.push_back(*it);
    }
    // default position is the last state
    result.states.push_back(defaultPos);
    return result;
  }

  /** Gets the last set of arm angles from this motion's states. */
  ArmAngles& getTargetState()
  {
    return *states.rbegin();
  },

  (ArmKeyFrameRequest::ArmKeyFrameId) id, /** Unique id of this motion. */
  (std::vector<ArmAngles>) states,    /** Array of states to move the arm to */
});

/**
 * Class to represent an Arm Motion. It consists of an array of
 * angle+stiffness settings to command consecutively to an arm. This class
 * is used by the ArmKeyFrameEngine to read definitions of arm motions from
 * a configuration file.
 * @author <a href="mailto:simont@tzi.de">Simon Taddiken</a>
 */

#pragma once

#include "Representations/MotionControl/ArmKeyFrameRequest.h"
#include "Math/Angle.h"
#include "Platform/BHAssert.h"
#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include <array>
#include <vector>

STREAMABLE(ArmKeyFrameMotion,
{
  ENUM(ArmInterpolation,
  {,
    linear,
    maxToZero,
    zeroToMax,
    zeroToMaxToZero,
  });

  /**
   * Represents one single state that an arm should reach. Consists of
   * target angles to be reached, stiffness data and amount of motion steps
   * defining the duration to reach the target angles.
   */
  STREAMABLE(ArmAngles,
  {,
    (std::array<Angle, Joints::numOfArmJoints>)({}) angles, /**< Target angles for all arm joints. */
    (std::array<int, Joints::numOfArmJoints>)({}) stiffness, /**< Stiffness data to set while targeting the above angles. */
  });

  STREAMABLE_WITH_BASE(ArmKeyFrameState, ArmAngles,
  {,
    (float)(1000.f) duration, /**< Duration in ms for reaching the target angles from the current position. */
    (ArmInterpolation)(zeroToMaxToZero) interpolation, /**< Interpolation method to use for this state. */
  });

  /**
   * Creates a new arm motion representing the reverse motion of this one. That is
   * it will contain the same array of target states but in reverse order with the
   * provided state as target state. Intended use is to reverse any arm motion to
   * reach back to the default position of the arm, that's why the default position
   * must be passed to this method. It will be appended as target state to the
   * created motion.
   * @param defaultPos Angle definition of the arm's default position.
   * @return A new ArmKeyFrameMotion targeting the arm's default position.
   */
  ArmKeyFrameMotion reverse(const ArmAngles& defaultPos) const
  {
    ASSERT(id != ArmKeyFrameRequest::reverse); // reverse motion not reversible
    ASSERT(!states.empty());

    ArmKeyFrameMotion result;
    result.id = ArmKeyFrameRequest::reverse;

    // add states in reverse order, but with the angles/stiffnesses shifted by one state
    for(auto it = states.rbegin(); it != states.rend(); ++it)
    {
      auto& state = result.states.emplace_back();
      static_cast<ArmAngles&>(state) = std::next(it) == states.rend() ? defaultPos : *std::next(it);
      state.duration = it->duration;
      switch(it->interpolation)
      {
        case maxToZero:
          state.interpolation = zeroToMax;
          break;
        case zeroToMax:
          state.interpolation = maxToZero;
          break;
        default:
          state.interpolation = it->interpolation;
          break;
      }
    }
    return result;
  }

  /** Gets the last set of arm angles from this motion's states. */
  const ArmAngles& getTargetState() const
  {
    ASSERT(!states.empty());
    return states.back();
  },

  (ArmKeyFrameRequest::ArmKeyFrameId) id, /**< Unique id of this motion. */
  (std::vector<ArmKeyFrameState>) states, /**< Array of states to move the arm to. */
});

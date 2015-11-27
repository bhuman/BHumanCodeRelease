/**
 * Request for the key frame engine.
 * @author <a href="mailto:simont@tzi.de>Simon Taddiken</a>
 */
#pragma once
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Arms.h"
#include "Tools/Enum.h"

/**
 * Class that represents the possible arm motions that can be requested from
 * the robot.
 */
STREAMABLE(ArmKeyFrameRequest,
{
  /** Existing arm motions. Ordering must match ordering in armMotionEngine.cfg */
  ENUM(ArmKeyFrameId,
  {,
    // IF U TOUCH THIS MAKE SURE U LOOKING IN THE MOTIONCOMBINATOR AND TAKE CARE TO SAVE THE ARMS CORRECTLY IN A FALLING CASE
    useDefault,  /**< No explicit arm motion, so WalkingEngine's arm angles will be used */
    back,        /**< Move arm to the back */
    keeperStand, /**< Arm position for the keeper when guarding the goal */
    reverse,     /**< Reverse current arm keyframe motion */
  });

  STREAMABLE(Arm,
  {,
    ((ArmKeyFrameRequest) ArmKeyFrameId)(useDefault) motion, /**< Motion to execute */
    (bool)(false) fast, /**< Whether states should not be interpolated */
  }),

  (Arm[Arms::numOfArms]) arms,
});

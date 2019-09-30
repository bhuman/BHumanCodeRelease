/**
 * @file Representations/MotionControl/ArmMotionSelection.h
 * This file declares a struct that represents the arm motions actually selected based on the constraints given.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/Streamable.h"
#include "ArmKeyFrameRequest.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"

#include <array>

/**
 * @struct ArmMotionSelection
 * A struct that represents the arm motions actually selected based on the constraints given.
 */
STREAMABLE(ArmMotionSelection,
{
  ENUM(ArmMotion,
  {,
    walkArms,
    kickArms,
    specialActionArms,
    standArms,
    getUpArms,
    fallArms,

    firstNonBodyMotion,
    clearS = firstNonBodyMotion, //assert same order as ArmMotionRequest
    keyFrameS,
    keyPoseS,
    pointAtS,
  });

  ArmMotionSelection()
  {
    targetArmMotion.fill(specialActionArms);
    armRatios[Arms::left].fill(0.f);
    armRatios[Arms::right].fill(0.f);
    armRatios[Arms::left][specialActionArms] = 1;
    armRatios[Arms::right][specialActionArms] = 1;
  },

  (ENUM_INDEXED_ARRAY(ArmMotionSelection::ArmMotion, Arms::Arm)) targetArmMotion, /**< The armmotion that is the destination of the current arminterpolation per arm. */
  (ENUM_INDEXED_ARRAY(ENUM_INDEXED_ARRAY(float, ArmMotion), Arms::Arm)) armRatios, /**< The current ratio of each armmotion in the final joint request, for each arm. */
  (ArmKeyFrameRequest) armKeyFrameRequest, /**< The key frame request per arm, if it is an active armmotion. */
});

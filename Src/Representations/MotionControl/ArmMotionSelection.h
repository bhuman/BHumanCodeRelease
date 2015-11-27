/**
 * @file Representations/MotionControl/ArmMotionSelection.h
 * This file declares a struct that represents the arm motions actually selected based on the constraints given.
 * @author Jesse Richter-Klug
 */

#pragma once

#include "ArmMotionRequest.h"
#include <cstring>

/**
 * @struct ArmMotionSelection
 * A struct that represents the arm motions actually selected based on the constraints given.
 */
STREAMABLE(ArmMotionSelection,
{
  int rightArmRatiosOffset;
  ArmMotionSelection()
  {
    memset(armRatios, 0, sizeof(armRatios));
    memset(targetArmMotion, ArmMotionRequest::none, sizeof(targetArmMotion));
    armRatios[ArmMotionRequest::none] = 1;
    armRatios[ArmMotionRequest::numOfArmMotions + ArmMotionRequest::none] = 1;
    rightArmRatiosOffset = ArmMotionRequest::numOfArmMotions;
  },

  ((ArmMotionRequest) ArmMotion[Arms::numOfArms]) targetArmMotion, /**< The armmotion that is the destination of the current arminterpolation per arm */
  (float[Arms::numOfArms * ArmMotionRequest::numOfArmMotions]) armRatios, /**< The current ratio of each armmotion in the final joint request, for each arm. */
  (ArmKeyFrameRequest) armKeyFrameRequest, /**< The key frame request per arm, if it is an active armmotion. */
});

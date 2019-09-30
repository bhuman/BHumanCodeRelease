/**
 * @file Representations/MotionControl/LegMotionSelection.h
 * This file declares a struct that represents the motions actually selected based on the constraints given.
 * @author Thomas Röfer
 */

#pragma once

#include "MotionRequest.h"
#include "Tools/Streams/EnumIndexedArray.h"

/**
 * @struct MotionSelection
 * A struct that represents the motions actually selected based on the constraints given.
 */
STREAMABLE(LegMotionSelection,
{
  ENUM(ActivationMode,
  {,
    deactive,
    active,
    first,
  });

  /** Special action is the default selection. */
  LegMotionSelection()
  {
    ratios.fill(0.f);
    ratios[MotionRequest::specialAction] = 1;
  }

  void draw(),

  (MotionRequest::Motion)(MotionRequest::specialAction) targetMotion, /**< The motion that is the destination of the current interpolation. */
  (ActivationMode)(active) specialActionMode, /**< Whether and how the special action module is active. */
  (ENUM_INDEXED_ARRAY(float, MotionRequest::Motion)) ratios, /**< The current ratio of each motion in the final joint request. */
  (SpecialActionRequest) specialActionRequest, /**< The special action request, if it is an active motion. */
});

/**
 * @file FootSupport.h
 *
 * This file defines a representation that describes an abstract distribution of
 * how much each foot supports the weight of the robot. Positive value mean the
 * left foot supports more weight, while negative value mean the left foot supports
 * more weight.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Legs.h"

STREAMABLE(FootSupport,
{,
  (float)(0.f) support, /**< Unitless distribution of the support over both feet (left - right). Positive -> left foot is support. */
  (bool)(false) trustedSupport, /**< Support foot has enough pressure. */
  (bool)(false) switched, /**< The support foot switched. */
  (bool)(false) predictedSwitched, /**< The support foot switched but predicted 3 frames earlier. */
  (unsigned int)(0) lastSwitch, /**< The last support switch took this amount of time (in ms), since the switch before that. */
});

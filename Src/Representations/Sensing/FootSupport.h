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

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/EnumIndexedArray.h"
#include "Tools/RobotParts/Legs.h"

STREAMABLE(SolePressureInfo,
{,
  (unsigned int)(0) forwardPressure, // Last time stamp the toe had pressure
  (unsigned int)(0) backwardPressure, // Last time stamp the heel had pressure
  (unsigned int)(0) hasPressure, // Last time stamp when foot had pressure
  (unsigned int)(0) hasPressureSince, // Time stamp the last time the the foot regained pressure
});

STREAMABLE(FootSupport,
{,
  (float)(0.f) support, /** Unitless distribution of the support over both feet (left - right). Positiv -> left foot is support. */
  (bool)(false) switched, /** The support foot switched. */
  (unsigned int)(0) lastSwitch,
  (bool)(false) predictedSwitched, /** The support foot switched but predicted 3 frames earlier. */
  (float)(0.3f) minPressure, /** Min Pressure needed to allow foot support switches. */
  (unsigned int) timeSinceLastUpdate, /** Time stamp when footSupport was last updated. */
  (ENUM_INDEXED_ARRAY(SolePressureInfo, Legs::Leg)) footPressure,
  (bool)(false) isCalibrated, /**< Min pressure of the FSRs are calibrated. */
});

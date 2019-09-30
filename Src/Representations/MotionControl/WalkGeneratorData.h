/**
 * @file WalkGeneratorData.h
 * This file contains information about the walking
 *
 * @author <a href="mailto:jesse@tzi.de">Philip Reichenberg/a>
 */

#pragma once

#include "Tools/Math/Angle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"

STREAMABLE(WalkGeneratorData,
{
  FUNCTION(void(const bool predictedStep)) updateCounter;
  FUNCTION(void(const Pose2f& speed, const Angle turn, const float forward, const Angle left)) updateWalkValues,

  (Pose2f) speed, //current walk speed
  (Angle) turn, //current walk turn value
  (float) forward, // current walk forward value
  (Angle) left, // current walk left value
  (int) usedPredictedSwitch, // counter for how many predicted foot support switches were made
});

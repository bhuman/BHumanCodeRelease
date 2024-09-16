/**
 * @file WalkStepData.h
 * This file contains information about the walking
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Math/Angle.h"
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Function.h"

STREAMABLE(WalkStepData,
{
  FUNCTION(void(const bool predictedStep)) updateCounter;
  FUNCTION(void(const Pose2f& stepTarget, const float stepDuration, const bool isLeftPhase)) updateWalkValues,

  (bool)(false) isLeftPhase,
  (Pose2f) stepTarget, //current walk value
  (float)(250.f) stepDuration, // current step duration
  (int)(0) usedPredictedSwitch, // counter for how many predicted foot support switches were made
  (unsigned)(0) lastUpdate, // time stamp of last update
});

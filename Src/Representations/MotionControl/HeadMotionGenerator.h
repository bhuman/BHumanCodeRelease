/**
 * @file HeadMotionGenerator.h
 *
 * This file declares a struct that generates head joint angles.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Streaming/Function.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(HeadMotionGenerator,
{
  FUNCTION(void(bool setJoints, JointRequest& jointRequest, HeadMotionInfo& headMotionInfo,
                const MotionRequest& motionRequest, const OdometryData& odometry)) calcJoints,
});

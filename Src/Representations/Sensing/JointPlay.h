/**
 * @file JointPlay.h
 * @author Philip Reichenberg
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"
#include "RobotParts/Joints.h"

STREAMABLE(JointPlay,
{,
  (float)(1.f) qualityOfRobotHardware, // 1.f means good robot, 0.f means bad robot // TODO right know it is the highest value
});

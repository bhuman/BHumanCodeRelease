/**
 * @file NaoQiImageInfo.h
 * This representation holds information about whether the correct image was used to flash the robot.
 * Otherwise the robot can not be used as all sensor data gives the value 0.
 * @author Philip Reichenberg
 */

#pragma once

#include "RobotParts/Joints.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/EnumIndexedArray.h"

STREAMABLE(NaoQiImageInfo,
{,
  (bool)(false) isCorrectImage,
});

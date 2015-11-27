/**
 * @file ArmMotionInfo.h
 * Definition of struct ArmMotionInfo.
 * @author Jesse Richter-Klug
 */

#pragma once

#include "ArmMotionRequest.h"

/**
 * @struct ArmMotionInfo
 * The executed motion request and additional information about the motions which are executed by the Motion process.
 */
STREAMABLE_WITH_BASE(ArmMotionInfo, ArmMotionRequest,
{,
});

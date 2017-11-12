/**
 * @file OdometryOffset.h
 *
 * This file defines a representation that contains the odometry offset.
 *
 * @author Jonas Kuball
 */

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE_WITH_BASE(OdometryOffset, Pose2f,
{,
});

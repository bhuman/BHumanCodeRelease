/**
 * @file OdometryData.h
 * Contains the OdometryData struct.
 */

#pragma once

#include "Math/Pose2f.h"

/**
 * @struct OdometryData
 * OdometryData contains an approximation of overall movement the robot has done.
 * @attention Only use differences of OdometryData at different times.
 * Position in mm
 */
STREAMABLE_WITH_BASE(OdometryData, Pose2f,
{,
});

STREAMABLE_WITH_BASE(OdometryDataPreview, OdometryData,
{,
  (Pose2f) odometryChange, /**< The change in the odometry compared to last frame. */
});

STREAMABLE_WITH_BASE(MotionOdometryData, OdometryData,
{,
});

STREAMABLE_WITH_BASE(OtherOdometryData, OdometryData,
{,
});

STREAMABLE_WITH_BASE(OdometryTranslationRequest, OdometryData,
{,
});

/**
 * @struct GroundTruthOdometryData
 * Contains an observed overall movement the robot has done.
 */
STREAMABLE_WITH_BASE(GroundTruthOdometryData, OdometryData,
{,
});

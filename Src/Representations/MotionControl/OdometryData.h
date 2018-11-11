/**
 * @file OdometryData.h
 * Contains the OdometryData struct.
 * @author Max Risler
 */

#pragma once

#include "Tools/Math/Pose2f.h"

/**
 * @struct OdometryData
 * OdometryData contains an approximation of overall movement the robot has done.
 * @attention Only use differences of OdometryData at different times.
 * Position in mm
 */
STREAMABLE_WITH_BASE(OdometryData, Pose2f,
{,
});

/**
 * @struct GroundTruthOdometryData
 * Contains an observed overall movement the robot has done.
 */
STREAMABLE_WITH_BASE(GroundTruthOdometryData, OdometryData,
{,
});

/**
 * @file GlobalPoseMeasurement.h
 *
 * This file declares a representation of a measurement of a pose on the field.
 * The conceptual idea is that this representation should be valid if, given the percepts of
 * a single frame (or perhaps a combined upper and lower frame), a unique pose (up to
 * field symmetry) can be derived (including a covariance matrix). It must be determined
 * elsewhere if this pose should be mirrored.
 *
 * This representation could also contain "references" to the percepts that have created it,
 * e.g. for drawings or to ensure in the localization that the same percept cannot be used
 * twice per frame.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Eigen.h"
#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(GlobalPoseMeasurement,
{
  void draw() const,

  (bool)(false) isValid, /**< Indicates that a unique pose (up to symmetry) could be derived and is stored below. */
  (Pose2f) poseOnField, /**< The pose "measurement". In contrast to other "onField" variables in Perception (which are on the floor, but still relative to the robot), this is relative to the "global" field coordinate system. */
  (Matrix3f)((Matrix3f() << 1.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 1.f).finished()) covarianceOnField, /**< A covariance matrix of the pose "measurement" (also in "global" field coordinates). */
});

/**
 * @file TeamPose.h
 *
 * TODO: Description
 *
 * @author <a href="mailto:flomaass@informatik.uni-bremen.de">Florian Maaﬂ</a>
 */
#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose2f.h"

STREAMABLE(TeamPose,
{
  TeamPose() = default;

  /** Verifies that the corrected robot pose and the covariance contain valid values. */
  void verify() const;
  /** Draws the corrected robot pose to the field view. */
  void draw() const,

  (bool)(false) update,  /**< whether to integrate covariance and pose to self locator or not */
  (Matrix3f) covariance, /**< use if update is true */
  (Pose2f) pose,         /**< use if update is true */
});

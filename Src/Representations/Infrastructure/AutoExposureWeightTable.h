/**
 * @file AutoExposureWeightTable.h
 *
 * This file implements a representation to describe how the camera is using the image reagions to calculate the auto exposure.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(AutoExposureWeightTable,
{
  void draw() const;
  void verify() const,

  // cells are allowed to have values from 0 to 100 (inclusive)
  (Eigen::Matrix<uint8_t, 5, 5>) table,
});

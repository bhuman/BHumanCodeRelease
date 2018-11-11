/**
 * @file AutoExposureWeightTable.h
 *
 * This file implements a representation to describe how the camera is using the image reagions to calculate the auto exposure.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/EnumIndexedArray.h"

using Matrix5uc = Eigen::Matrix<uint8_t, 5, 5, Eigen::RowMajor>;

STREAMABLE(AutoExposureWeightTable,
{
  void draw() const;
  void verify() const,

  // cells are allowed to have values from 0 to 100 (inclusive)
  (ENUM_INDEXED_ARRAY(Matrix5uc, CameraInfo::Camera)) tables,
});

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

STREAMABLE(AutoExposureWeightTable,
{
  static constexpr int width = 4;
  static constexpr int height = 4;
  static constexpr uint8_t maxWeight = 15;
  using Table = Eigen::Matrix<uint8_t COMMA height COMMA width COMMA Eigen::RowMajor>;

  void draw() const;
  void verify() const,

  // cells are allowed to have values from 0 to maxWeight (inclusive)
  (ENUM_INDEXED_ARRAY(Table, CameraInfo::Camera)) tables,
});

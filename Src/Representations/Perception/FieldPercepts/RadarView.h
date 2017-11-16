/**
 * @file RadarView.h
 *
 * @author apag
 */

#pragma once

#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"

/**
 * @struct RadarView
 */
STREAMABLE_WITH_BASE(RadarView, FieldLines,
{
  /**
   * @struct EqLines
   *
   * This struct represents a list of equivalent lines. (TODO currently not used)
   */
  STREAMABLE(EqLines,
  {,
    (std::vector<FieldLines::Line>) eqLines,
  });
  void draw() const,
  (std::vector<RadarView::EqLines>) sortLines,
});

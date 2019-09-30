/**
 * @author Alexis Tsogias
 */

#pragma once

#include <vector>
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FieldBoundary,
{
  using InImage = std::vector<Vector2i>; ///< Type for the boundary in image coordinates.
  using InField = std::vector<Vector2f>; ///< Type for the boundary in field coordinates.

  /**
   * Draws some DebugDrawings
   *
   * Modifiable:
   *   representation:FieldBoundary:SelectedCandidate - Set the candidate which shall be drawn
   *
   * DebugDrawings:
   *   representation:FieldBoundary:BoundarySpots
   *   representation:FieldBoundary:ConvexBoundary
   *   representation:FieldBoundary:Image - The field final boundary in the image.
   *   representation:FieldBoundary:Field - The field final boundary on the field.
   */
  void draw() const;

  /**
   * Verify that coordinates are valid.
   * Not created in Release to avoid warning about unused variable.
   */
  void verify() const
  {
    for(const Vector2f& p : boundaryOnField)
    {
      static_cast<void>(p); // suppress warning in Release
      ASSERT(std::isfinite(p.x()) && std::isfinite(p.y()));
    }
  }

  /**
   * Returns the y coordinate of the field boundary at the specified x coordiante in the current image.
   */
  int getBoundaryY(int x) const,

  (InField) boundaryOnField,   ///< The boundary projected to the field in relative coordinates.
  (InImage) boundaryInImage,   ///< The boundary in image coordinates.
  (bool)(false) isValid,       ///< True if a boundary could be detected.
});

STREAMABLE_WITH_BASE(OtherFieldBoundary, FieldBoundary, {,});

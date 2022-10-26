/**
 * @author Alexis Tsogias
 */

#pragma once

#include <vector>
#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

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
    for([[maybe_unused]] const Vector2f& p : boundaryOnField)
      ASSERT(std::isfinite(p.x()) && std::isfinite(p.y()));
  }

  /**
   * Returns the y coordinate of the field boundary at the specified x coordinate in the current image.
   */
  int getBoundaryY(int x) const;

  /**
   * @param imageWidth The width of the current image.
   * @return The topmost y coordinate of the field boundary in the current image.
   */
  int getBoundaryTopmostY(int imageWidth) const,

  (InField) boundaryOnField,   ///< The boundary projected to the field in relative coordinates.
  (InImage) boundaryInImage,   ///< The boundary in image coordinates.
  (bool)(false) isValid,       ///< True if a boundary could be detected.
  (bool)(false) extrapolated,  ///< True if the boundary is constructed from a previous
  (bool)(false) odd,           ///< True if the boundary is odd/not good
});

STREAMABLE_WITH_BASE(OtherFieldBoundary, FieldBoundary, {,});

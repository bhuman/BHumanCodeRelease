/**
 * @author Alexis Tsogias
 */

#pragma once

#include <vector>
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(FieldBoundarySpots,
{
  STREAMABLE(Spot,
  {
    Spot() = default;
    Spot(Vector2i position, int rating),

    (Vector2i) position,
    (int) rating,
  });

  /**
   * Draws some DebugDrawings
   *
   * DebugDrawings:
   *   representation:FieldBoundary:BoundarySpots
   *   representation:FieldBoundary:ConvexBoundary
   */
  void draw() const,

  (std::vector<Spot>) spots,     ///< Spots on the boundary.
});

STREAMABLE(FieldBoundary2,
{
  using InImage = std::vector<Vector2i>; ///< Type for the boundary in image coordinates.
  using InField = std::vector<Vector2f>; ///< Type for the boundary in field coordinates.

  /**
   * Draws some DebugDrawings
   *
   * DebugDrawings:
   *   representation:FieldBoundary:Image - The field final boundary in the image.
   *   representation:FieldBoundary:Field - The field final boundary on the field.
   */
  void draw() const;

  /**
   * Returns the y coordinate of the field boundary at the specified x coordiante in the current image.
   */
  int getBoundaryY(int x) const;
  ,
  (bool)(false) isValid,       ///< True if a boundary could be detected. If false boundaryInImage and boundaryOnField will be empty
  (InImage) boundaryInImage,   ///< The boundary in image coordinates.
  (InField) boundaryOnField,   ///< The boundary projectet to the Field in relative coordinates.
});

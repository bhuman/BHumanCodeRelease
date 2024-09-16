/**
 * @file IntersectionsCandidates.h
 *
 * This file defines a representation that ...
 *
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 */

#pragma once

#include "Debugging/DebugDrawings.h"
#include "Streaming/AutoStreamable.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include <vector>

STREAMABLE(IntersectionCandidates,
{
  STREAMABLE(IntersectionCandidate,
  {
    IntersectionCandidate() = default;
    IntersectionCandidate(const IntersectionsPercept::Intersection::IntersectionType& t, const Vector2f& p, const Vector2f& ap, const Matrix2f& c, const Vector2f& d1,
                          const Vector2f& d2, unsigned l1, unsigned l2, const Vector2f& l1CloserEnd, const Vector2f& l2CloserEnd,
                          const Vector2f& l1FurtherEnd, const Vector2f& l2FurtherEnd, float dist, const Image<PixelTypes::GrayscaledPixel> patch)
    {
      type = t;
      pos = p;
      absPos = ap;
      cov = c;
      dir1 = d1;
      dir2 = d2;
      line1Index = l1;
      line2Index = l2;
      line1CloserEnd = l1CloserEnd;
      line2CloserEnd = l2CloserEnd;
      line1FurtherEnd = l1FurtherEnd;
      line2FurtherEnd = l2FurtherEnd;
      distance = dist;
      imagePatch = patch;
    },

    (IntersectionsPercept::Intersection::IntersectionType) type,
    (Vector2f) pos,              /**< The field coordinates of the intersection, relative to the robot */
    (Vector2f) absPos,           /**< The field coordinates of the intersection absolute */
    (Matrix2f) cov,              /**< The covariance of pos */
    (Vector2f) dir1,             /**< The first direction of the lines intersected (in field coordinates, relative to the robot) */
    (Vector2f) dir2,             /**< The second direction of the lines intersected (in field coordinates, realative to the robot) */
    (unsigned) line1Index,       /**< The first line of the intersection*/
    (unsigned) line2Index,       /**< The second line of the intersection*/
    (Vector2f) line1CloserEnd,   /**< The closer end of the first line to the intersection in field coordinates */
    (Vector2f) line2CloserEnd,   /**< The closer end of the second line to the intersection in field coordinates */
    (Vector2f) line1FurtherEnd,  /**< The further end of the first line to the intersection in field coordinates */
    (Vector2f) line2FurtherEnd,  /**< The further end of the second line to the intersection in field coordinates */
    (float) distance,            /**< The normalized distance between robot and intersection */
    (Image<PixelTypes::GrayscaledPixel>) imagePatch,
  });

  void draw() const,

  (std::vector<IntersectionCandidate>) intersections, /**< A list of all intersection candidates that were found in the current image */
});

inline void IntersectionCandidates::draw() const
{
  /*DEBUG_DRAWING("representation:IntersectionsPercept:field", "drawingOnField")
  {
    for(const IntersectionCandidate& intersection : intersections)
    {
      CROSS("representation:IntersectionsPercept:field", intersection.pos.x(), intersection.pos.y(), 60, 30, Drawings::solidPen, ColorRGBA::blue);
    }
  }*/
}

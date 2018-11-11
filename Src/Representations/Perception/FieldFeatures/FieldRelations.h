/**
 * @file FieldRelations.h
 * Declaration of a struct that represents intersection relations.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Streamable.h"

#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "FieldFeature.h"
#include "FieldMarker.h"

STREAMABLE(IntersectionRelations,
{
  STREAMABLE(IntersectionLineRelations,
  {
    IntersectionLineRelations(),

    (MarkedLine::LineMarker[MarkedIntersection::firstIntersectionMarkerOther]) lineMarkerOfDir1,
    (MarkedLine::LineMarker[MarkedIntersection::firstIntersectionMarkerOther]) lineMarkerOfDir2,
  });

  struct LineIntersectionRelations : public Streamable
  {
    LineIntersectionRelations() = default;
    LineIntersectionRelations(const FieldDimensions& theFieldDimensions);

    MarkedIntersection::IntersectionMarker intersections[MarkedLine::firstLineMarkerOther][4];
    float intersectionPositions[MarkedLine::firstLineMarkerOther][4];

  protected:
    void serialize(In* in, Out* out) override;
  };

  IntersectionRelations() = default;
  IntersectionRelations(const FieldDimensions& fd) : lineIntersectionRelations(fd) {}

  void propagateMarkedIntersection(const MarkedIntersection& start, const FieldLineIntersections& theFieldLineIntersections,
                                   const FieldLines& theFieldLines, FieldFeature& ff) const;
  void propagateMarkedLinePoint(const MarkedLine& start, const float linePoint, const Vector2f& fieldPoint,
                                const FieldLineIntersections& theFieldLineIntersections, const FieldLines& theFieldLines, FieldFeature& ff) const,

  (IntersectionLineRelations) intersectionLineRelations,
  (LineIntersectionRelations) lineIntersectionRelations,
});

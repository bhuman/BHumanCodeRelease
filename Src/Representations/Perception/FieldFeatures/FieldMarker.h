/**
 * @file FieldMarker.h
 * Declaration of representations to mark the field percepts (major lines and line-intersections).
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct MarkedLine
 * It defines a line that is marked.
 */
STREAMABLE(MarkedLine,
{
  /*
    ___________3__________
   |      4|______|5      |
   |           6          |
   |2                     |
   |                      |
   |___________1__________|

   */
  ENUM(LineMarker,
  {,
    midLine, // --> 1
    sideLine, // --> 2
    groundLine, // --> 3
    sidePenaltyL, // --> 4
    sidePenaltyR, // --> 5
    groundPenalty, // --> 6

    firstLineMarkerOther,
    otherSideLine = firstLineMarkerOther,
    otherGroundLine,
    otherSidePenaltyL,
    otherSidePenaltyR,
    otherGroundPenalty,
  });
  static LineMarker mirror(const LineMarker marker);
  LineMarker mirror() const;

  MarkedLine() = default;
  MarkedLine(unsigned line, LineMarker marker);
  void draw() const,

  (unsigned) lineIndex,
  (LineMarker)(numOfLineMarkers) marker,
});

inline MarkedLine::MarkedLine(unsigned line, LineMarker marker) : lineIndex(line), marker(marker) {}

/**
 * @struct MarkedIntersection
 * It defines an intersection that is marked.
 */
STREAMABLE(MarkedIntersection,
{
  /*
           STL    STR
  BLL _______v______v_______ BLR
    |       |______|       |
    |    SLL        SLR    |
    |                      |
    |                      |
    |______________________|
  BT                         otherBT

   */
  /**
   * marker name components: (S/B)(L/T)(L/R)
   *  (Small/Big)(L-intersection, T-intersection)(Left/Right)
   */
  ENUM(IntersectionMarker,
  {,
    BT,
    STL,
    STR,
    BLL,
    SLL,
    SLR,
    BLR,

    firstIntersectionMarkerOther,
    otherBT = firstIntersectionMarkerOther,
    otherSTL,
    otherSTR,
    otherBLL,
    otherSLL,
    otherSLR,
    otherBLR,
  });
  static IntersectionMarker mirror(const IntersectionMarker& marker);
  IntersectionMarker mirror() const;

  MarkedIntersection() = default;
  MarkedIntersection(unsigned intersection, IntersectionMarker marker);

  void draw() const,

  (unsigned) intersectionIndex,
  (IntersectionMarker)(numOfIntersectionMarkers) marker,
});

inline MarkedIntersection::MarkedIntersection(unsigned intersection, IntersectionMarker marker)
  : intersectionIndex(intersection), marker(marker) {};

/**
 * @struct MarkedPoint
 * It defines a point that is marked.
 */
STREAMABLE(MarkedPoint,
{
  ENUM(PointMarker,
  {,
    midCircle, //Circle
    penaltyMark, //PenaltyMark
    goalPostL, //left goal post
    goalPostR, //right goal post

    firstPointMarkerOther,
    otherpenaltyMark = firstPointMarkerOther,
    othergoalPostL,
    othergoalPostR,
  });
  static PointMarker mirror(const PointMarker& marker);
  PointMarker mirror() const;

  MarkedPoint() = default;
  MarkedPoint(const Vector2f& p, PointMarker marker, bool outOfCurrentFrame = true);

  void draw() const,//TODO

  (Vector2f)(Vector2f()) point,
  (PointMarker)(numOfPointMarkers) marker,
  (bool)(false) outOfCurrentFrame,
});

inline MarkedPoint::MarkedPoint(const Vector2f& p, PointMarker marker, bool outOfCurrentFrame)
  : point(p), marker(marker), outOfCurrentFrame(outOfCurrentFrame) {}

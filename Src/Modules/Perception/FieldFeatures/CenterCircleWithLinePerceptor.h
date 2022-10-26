/**
* @file CenterCircleWithLinePerceptor.h
*
* Declaration of a module that tries to find the combination of
* the center circle and the center line crossing the circle.
* in the current field percepts. This is the combination
* that this modul is looking for:
*
*                   _____
*                  *     *      (ugly, I know)
*                 /       \
*    ------------+----+----+------------
*                 \       /
*                  *     *
*                   -----
*
* @author Tim Laue
*/

#pragma once

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/FieldFeatures/CenterCircleWithLine.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Framework/Module.h"

MODULE(CenterCircleWithLinePerceptor,
{,
  REQUIRES(FrameInfo),
  REQUIRES(FieldLines),
  REQUIRES(CirclePercept),
  REQUIRES(Odometer),
  REQUIRES(FieldDimensions),
  PROVIDES(CenterCircleWithLine),
  DEFINES_PARAMETERS(
  {,
    (int)(80) bufferTimeCenterCircle,             /**< Use old center circles not longer than for this number of milliseconds */
    (float)(1500.f) minimumLengthOfPerceivedLine, /**< A line must be at least this long to be considered as the center line */
    (float)(200.f) maxLineDeviationFromCenter,    /**< The distance between the center of the center circle and the line is not allowed to be larger than this value */
    (bool)(false) detectCircleByCrossings,        /**< If set to true, a combination of close line crossings can be interpreted as the intersections between circle and center line */
  }),
});

/**
* A module that tries to find the combination of
* the center circle and the center line crossing the circle.
* in the current field percepts.
*/
class CenterCircleWithLinePerceptor : public CenterCircleWithLinePerceptorBase
{
  Vector2f centerCirclePosition;              /**< Buffered position of center circle */
  unsigned timeWhenCenterCircleLastSeen;      /**< Point of time when the center circle was seen for the last time */
  Geometry::Line centerLine;                  /**< If a center line was found, it becomes stored here (coordinates on field, relative to robot) */

  void update(CenterCircleWithLine& centerCircleWithLine) override;

  /** Tries to find a field line that seems to be the center line.
   *  If a line was found, its start and end point are written to the lineStart and lineEnd members.
   *  @return true, if a line was found. false otherwise.
   */
  bool findIntersectingCenterLine();

  /** Computes a pose based on the center circle and the center line and fills the representation
   * @param centerCircleWithLine The representation set by this method.
   */
  void computePose(CenterCircleWithLine& centerCircleWithLine);

public:
  /** Constructor */
  CenterCircleWithLinePerceptor();
};

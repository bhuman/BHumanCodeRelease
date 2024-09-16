/**
* @file CenterCircleWithLinePerceptor.h
*
* Declaration of a module that tries to find the combination of
* the center circle and the halfway line crossing the circle.
* in the current field percepts. This is the combination
* that this module is looking for:
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
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/FieldFeatures/CenterCircleWithLine.h"
#include "Representations/Perception/FieldPercepts/CirclePercept.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Framework/Module.h"

MODULE(CenterCircleWithLinePerceptor,
{,
  REQUIRES(CirclePercept),
  REQUIRES(FrameInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLines),
  REQUIRES(Odometer),
  REQUIRES(WorldModelPrediction),
  PROVIDES(CenterCircleWithLine),
  DEFINES_PARAMETERS(
  {,
    (int)(80) bufferTimeCenterCircle,             /**< Use old center circles not longer than for this number of milliseconds */
    (float)(450.f) minimumLengthOfPerceivedLine,  /**< A line must be at least this long to be considered as the halfway line */
    (float)(1000.f) maxExtraCheckingLineLength,   /**< Lines that are shorter than this threshold have a stricter check for their distance to the center circle's center */
    (float)(200.f) maxLineDeviationFromCenter,    /**< The distance between the center of the center circle and the line is not allowed to be larger than this value */
    (Angle)(80_deg) maxAbsAngleAlpha,             /**< Threshold to avoid some lines that are "too vertical" and might result from false positives inside robots */
  }),
});

/**
* A module that tries to find the combination of
* the center circle and the halfway line crossing the circle.
* in the current field percepts.
*/
class CenterCircleWithLinePerceptor : public CenterCircleWithLinePerceptorBase
{
  Vector2f centerCirclePosition;              /**< Buffered position of center circle */
  Matrix2f centerCircleCovariance;            /**< Buffered covariance of center circle */
  unsigned timeWhenCenterCircleLastSeen;      /**< Point of time when the center circle was seen for the last time */
  Geometry::Line halfwayLine;                 /**< If a halfway line was found, it becomes stored here (coordinates on field, relative to robot) */
  Matrix2f halfwayLineCov;                    /**< The covariance of the determined halfway line, copied from line percept */

  void update(CenterCircleWithLine& centerCircleWithLine) override;

  /** Tries to find a field line that seems to be the halfway line.
   *  If a line was found, its start and end point are written to the lineStart and lineEnd members.
   *  @return true, if a line was found. false otherwise.
   */
  bool findIntersectingHalfwayLine();

  /** Computes a pose based on the center circle and the halfway line and fills the representation
   * @param centerCircleWithLine The representation set by this method.
   */
  void computeFeature(CenterCircleWithLine& centerCircleWithLine);

public:
  /** Constructor */
  CenterCircleWithLinePerceptor();
};

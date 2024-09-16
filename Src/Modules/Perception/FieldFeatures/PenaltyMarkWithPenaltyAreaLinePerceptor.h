/**
* @file PenaltyMarkWithPenaltyAreaLinePerceptor.h
*
* Declaration of a module that tries to find the combination of
* a penalty mark and the line of the penalty area that is
* closest to it (the line between penalty mark and the field's
* halfway line) in the current field percepts. This is the combination
* that this module is looking for:
*
*                  (goal)
*-----------------------------------------
*          |                   |
*          |         + (mark)  |
*          ===================== (<- this line)
*
* @author Tim Laue
*/

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Modeling/WorldModelPrediction.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/FieldFeatures/PenaltyMarkWithPenaltyAreaLine.h"

MODULE(PenaltyMarkWithPenaltyAreaLinePerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FieldLines),
  REQUIRES(FrameInfo),
  REQUIRES(Odometer),
  REQUIRES(PenaltyMarkPercept),
  REQUIRES(WorldModelPrediction),
  PROVIDES(PenaltyMarkWithPenaltyAreaLine),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) minimumLineLength,   /**< The penalty area line has to be at least this long */
    (float)(150.f)  maximumDeviation,    /**< Maximum tolerated deviation between modeled distance and perceived distance */
    (int)(40) bufferTimePenaltyMark,     /**< Use old penalty marks not longer than for this number of milliseconds */
  }),
});

/**
 * A module that tries to find the combination of
 * a penalty mark and the line of the penalty area that is
 * closest to it (the line between penalty mark and the field's
 * halfway line) in the current field percepts.
 */
class PenaltyMarkWithPenaltyAreaLinePerceptor : public PenaltyMarkWithPenaltyAreaLinePerceptorBase
{
  float modelDistancePenaltyMarkToLine;      /**< The original distance between penalty mark and line, as defined by the official field model */
  Vector2f lineStart;                        /**< If a line was found, its start position becomes stored here (coordinates on field, relative to robot) */
  Vector2f lineEnd;                          /**< If a line was found, its end position becomes stored here (coordinates on field, relative to robot) */
  Matrix2f lineCov;                          /**< The covariance of the determined halfway line, copied from line percept */
  Vector2f penaltyMarkPosition;              /**< Buffered position of penalty mark */
  Matrix2f penaltyMarkCovariance;            /**< Buffered covariance of penalty mark */
  unsigned timeWhenPenaltyMarkLastSeen;      /**< Point of time when the penalty mark was seen for the last time */

  /** Main method of this module
   * @param penaltyMarkWithPenaltyAreaLine The representation that is filled by this module
   */
  void update(PenaltyMarkWithPenaltyAreaLine& penaltyMarkWithPenaltyAreaLine) override;

  /** Tries to find a field line that seems to be the penalty area border line.
   *  If a line was found, its start and end point are written to the lineStart and lineEnd members.
   *  @return true, if a line was found. false otherwise.
   */
  bool findMatchingLine();

  /** Computes a pose based on the penalty mark and the line and fills the representation
   * @param penaltyMarkWithPenaltyAreaLine The representation set by this method.
   */
  void computeFeature(PenaltyMarkWithPenaltyAreaLine& penaltyMarkWithPenaltyAreaLine);

public:
  /** Constructor */
  PenaltyMarkWithPenaltyAreaLinePerceptor();
};

/**
 * @file SelfLocalizationHypotheses.h
 *
 * List of robot pose hypotheses that are currently evaluated by the self-localization module.
 * This information is not used as input for further computations but is logged and
 * helps during the debugging of localization problems.
 *
 * @author Tim Laue
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct SelfLocalizationHypotheses
 * A list of hypotheses currently tracked by the self-localization module.
 */
STREAMABLE(SelfLocalizationHypotheses,
{
  /**
   * @struct Hypothesis
   * A possible robot pose and some additional information about its precision / reliability.
   * To keep the amount of data small, only parts of the 3-dimensional covariance matrix are stored here.
   */
  STREAMABLE(Hypothesis,
  {,
    (Pose2f) pose,             /**< The pose in 2D (position + rotation) */
    (float) validity,          /**< The validity of the pose estimate (based on quality of recent matches with perceptions of the field) */
    (float) xVariance,         /**< Variance of the estimate in global field's x-direction */
    (float) yVariance,         /**< Variance of the estimate in global field's y-direction */
    (float) xyCovariance,      /**< Covariance of x/y uncertainty */
    (float) rotVariance,       /**< Variance of rotation estimate */
  });

  /** Draws this representation. */
  void draw() const,

  /** A list of hypotheses */
  (std::vector<Hypothesis>) hypotheses,
});

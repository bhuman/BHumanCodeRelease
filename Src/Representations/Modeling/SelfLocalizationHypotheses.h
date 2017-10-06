/**
 * @file SelfLocalizationHypotheses.h
 *
 * List of robot pose hypotheses that are currently evaluated by the self-localization module
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"

/**
 * @struct SelfLocalizationHypotheses
 * A list of currently evaluated hypotheses
 */
STREAMABLE(SelfLocalizationHypotheses,
{
  /**
   * @struct Hypothesis
   * A possible robot pose and some additional information
   */
  STREAMABLE(Hypothesis,
  {,
    (Pose2f) pose,             /** the pose in 2D (position + rotation) */
    (float) validity,
    (float) xVariance,
    (float) yVariance,
    (float) xyCovariance,
    (float) rotVariance,
  });

  /** Draws this representation. */
  void draw() const,

  /** A list of hypotheses*/
  (std::vector<Hypothesis>) hypotheses,
});

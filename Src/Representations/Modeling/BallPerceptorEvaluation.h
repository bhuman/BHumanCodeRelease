#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"

/**
 * A representation that contains some statistics for given BallPercepts.
 * The current structure will need a special test setup for comparison reasons.
 *
 * @author Alexander Stöwing <stoewing@uni-bremen.de>
 */

STREAMABLE(BallPerceptorEvaluation,
{
  BallPerceptorEvaluation() = default,

  (float)(0.f) seenPercentage,           //Over all Frames, how many contained an percept (in percent)
  (float)(0.f) guessedFalsePositives,    //How many percepts are possible false positives (in percent)
  (float)(0.f) averageDifferenceUpper,   //Do the percepts in the upperCamera jump a lot
  (float)(0.f) averageDifferenceLower,   //Do the percepts in the lowerCamera jump a lot
  (float)(0.f) maximumPerceptDistance,   //The maximum distance a ball was found
  (float)(0.f) minimumPerceptDistance,   //The minimum distance a ball was found
  (float)(0.f) averagePerceptDistance,   //The average distance the percepts have
});

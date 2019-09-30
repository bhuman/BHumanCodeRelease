/**
 * @file FrictionLearner.h
 *
 * This file declares a module that estimates the ball friction coefficient.
 *
 * @author Tim Laue
 * @author Felix Wenk
 *
 * This module is intended to learn the ball friction coefficient. It assumes a linear model
 * for ball deceleration:
 *
 *   s = v * t + 0.5 * a * t^2  with v = ball velocity, a = ballFriction, s = rolled distance
 *
 * You can use this module as follows.
 * 1. Start it: mr DummyRepresentation FrictionLearner
 * 2. Roll the ball once through the robot's field of view
 *    - start and end point do not matter, but starting and ending outside the field of view *might* be better
 *    - avoid collisions as well as places on the field that are not completely even
 * 3. Read the messages that are printed on the console ;-)
 * 4. Repeat 2. + 3. multiple times and filter out the outliers by yourself ;-)
 */

#pragma once

#include <vector>
#include "Tools/Module/Module.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Infrastructure/DummyRepresentation.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Math/Eigen.h"

MODULE(FrictionLearner,
{,
  REQUIRES(BallPercept),
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  PROVIDES(DummyRepresentation),
  DEFINES_PARAMETERS(
  {,
    (int)(1000) timeout,                 /** Time in ms. If there has not been observed any ball for timeout ms, the data collection phase ends. */
    (unsigned int)(20) minObservations,  /** Minimum number of ball observations that is required for optimization */
    (unsigned int)(8) offset,            /** Buffer index offset between two observations that are used as a pair for optimization*/
    (bool)(false) acceptGuessedBalls,    /** Incorporate guessed balls in friction estimation, if set to true */
  }),
});

/**
 * @class FrictionLearner
 *
 * This class implements a module that estimates the ball friction coefficient.
 */
class FrictionLearner : public FrictionLearnerBase
{
private:

  /** Internal structure for saving information about a perceived ball */
  struct BallObservation
  {
    /** Constructor */
    BallObservation(Vector2f p, unsigned int t): pos(p), time(t)
    {}

    Vector2f pos;      /**< The position on the field (relative to the robot)*/
    unsigned int time;  /**< The point of time (in ms) of the observation */
  };

  std::vector<BallObservation> balls;      /** List of observed balls */

  /** Main function that triggers the estimation process
   * @param dummy Nothing interesting at all
   */
  void update(DummyRepresentation& dummy) override;

  /** If enough data has been collected by the update method, this method is called for the actual computation.
   *  The output is printed to the console window.
   */
  void determineFrictionCoefficient();
};

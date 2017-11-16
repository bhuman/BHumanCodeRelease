/**
 * @file Representations/MotionControl/BallTrajectory.h
 * This file declares a struct that contains informations about the coarse ball trajectory
 * @author Bernd Poppinga
 * @author Markus Prinzler
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"

/**
 * @struct BallTrajectory
 * A struct that predicts the coarse ball trajectory.
 */
STREAMABLE(BallTrajectory,
{,
  (Vector2f) muBallPosition, // mean ball position relative to robot
  (Vector2f) muBallVelocity, // mean ball velocity relative to robot
  (Matrix2f) sigmaBallPosition, // cov ball position relative to robot
  (Matrix2f) sigmaBallVelocity, // cov ball velocity relative to robot
  (Vector2f) intersectionWithYAxis, // ball coordinates when it intersects y axis => to be removed
});

/**
 * @file FrictionLearner.cpp
 *
 * This file implements a module that estimates the ball friction coefficient.
 *
 * @author Tim Laue
 * @author Felix Wenk
 */

#include "FrictionLearner.h"

MAKE_MODULE(FrictionLearner, infrastructure)

void FrictionLearner::update(DummyRepresentation& dummy)
{
  if((theBallPercept.status == BallPercept::seen || (theBallPercept.status == BallPercept::guessed && acceptGuessedBalls))
     && theBallModel.estimate.velocity.norm() != 0.f)
    balls.push_back(BallObservation(theBallPercept.positionOnField, theFrameInfo.time));

  if(balls.size() != 0 && theFrameInfo.getTimeSince(balls.back().time) > timeout)
  {
    if(balls.size() >= minObservations + offset)
      determineFrictionCoefficient();
    else
      OUTPUT_TEXT("Not enough observations (" << static_cast<int>(balls.size()) << "/" << static_cast<int>(minObservations + offset) << ")");
    balls.clear();
  }
}

void FrictionLearner::determineFrictionCoefficient()
{
  OUTPUT_TEXT("Computing ball friction coefficient! Num of percepts: " << static_cast<int>(balls.size()));
  Eigen::MatrixX4f A((balls.size() - offset) * 2, 4);
  Eigen::VectorXf  z((balls.size() - offset) * 2);
  unsigned int t0 = balls[0].time;
  for(unsigned int i = 0; i < balls.size() - offset; i++)
  {
    const BallObservation& b1 = balls[i];
    const BallObservation& b2 = balls[i + offset];
    const auto dt    = (b2.time - b1.time) / 1000.f;   // in s
    const auto tToB1 = (b1.time - t0) / 1000.f;        // in s
    const auto dPos = (b2.pos - b1.pos) / 1000.f;      // in m
    z(i * 2)         = dPos.x();
    z(i * 2 + 1)     = dPos.y();
    A(i * 2, 0)      = dt;
    A(i * 2, 1)      = 0.f;
    A(i * 2, 2)      = 0.5f * dt * dt + dt * tToB1;
    A(i * 2, 3)      = 0.f;
    A(i * 2 + 1, 0)  = 0.f;
    A(i * 2 + 1, 1)  = dt;
    A(i * 2 + 1, 2)  = 0.f;
    A(i * 2 + 1, 3)  = 0.5f * dt * dt + dt * tToB1;
  }
  Vector4f x;
  x = (A.transpose() * A).inverse() * A.transpose() * z;
  OUTPUT_TEXT("Result:  v=(" << x(0) << "," << x(1) << ")   a=(" << x(2) << "," << x(3) << ")   abs(a)=" << std::sqrt(x(2) * x(2) + x(3) * x(3)));

  // Compute residuals and print error on console:
  Eigen::VectorXf residuals = A * x - z;
  float meanError = residuals.norm() / std::sqrt(static_cast<float>(balls.size() - offset));
  OUTPUT_TEXT("Mean error: " << meanError * 1000.f << " mm");
}

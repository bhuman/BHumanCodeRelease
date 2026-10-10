/**
 * @file FrictionLearner.cpp
 *
 * This file implements a module that estimates the ball friction coefficient.
 *
 * @author Tim Laue
 * @author Felix Wenk
 */

#include "FrictionLearner.h"

MAKE_MODULE(FrictionLearner);

void FrictionLearner::update(DummyRepresentation&)
{
  DECLARE_DEBUG_DRAWING("module:FrictionLearner:balls", "drawingOnField");
  if((theBallPercept.status == BallPercept::seen || (theBallPercept.status == BallPercept::guessed && acceptGuessedBalls))
     && theBallModel.estimate.velocity.norm() != 0.f)
    balls.push_back(BallObservation(theBallPercept.positionOnField, theFrameInfo.time));
  for(auto& b : balls)
  {
    CIRCLE("module:FrictionLearner:balls", b.pos.x(), b.pos.y(), theBallSpecification.radius, 1.f,
           Drawings::solidPen, ColorRGBA::white, Drawings::solidBrush, ColorRGBA::violet);
  }
  if(balls.size() != 0 && theFrameInfo.getTimeSince(balls.back().time) > timeout)
  {
    if(balls.size() >= minObservations + offset)
    {
      OUTPUT_TEXT("\nComputing ball friction coefficient! Num of percepts: " << static_cast<int>(balls.size()));
      OUTPUT_TEXT("\n *** Old : ***");
      determineFrictionCoefficient();
      OUTPUT_TEXT("\n *** New (full): ***");
      determineFrictionCoefficient2(0, static_cast<int>(balls.size()-1));
    /*  OUTPUT_TEXT("\n *** New (skip first 20%): ***");
      determineFrictionCoefficient2(static_cast<int>(balls.size()) / 5, static_cast<int>(balls.size()-1));
      OUTPUT_TEXT("\n *** New (skip last 20%): ***");
      determineFrictionCoefficient2(0, static_cast<int>(balls.size()) * 4 / 5);*/
      OUTPUT_TEXT("\n *** New (skip first and last 20%): ***");
      determineFrictionCoefficient2(static_cast<int>(balls.size()) / 5, static_cast<int>(balls.size()) * 4 / 5);
      OUTPUT_TEXT("\n\n");
    }
    else
    {
      OUTPUT_TEXT("Not enough observations (" << static_cast<int>(balls.size()) << "/" << static_cast<int>(minObservations + offset) << ")");
    }
    balls.clear();
  }
}

void FrictionLearner::determineFrictionCoefficient()
{
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
  x = A.colPivHouseholderQr().solve(z);

  OUTPUT_TEXT("Result:  v=(" << x(0) << "," << x(1) << ")   a=(" << x(2) << "," << x(3) << ")   abs(a)=" << std::sqrt(x(2) * x(2) + x(3) * x(3)));

  // Compute residuals and print error on console:
  Eigen::VectorXf residuals = A * x - z;
  float meanError = residuals.norm() / std::sqrt(static_cast<float>(balls.size() - offset));
  OUTPUT_TEXT("Mean error: " << meanError * 1000.f << " mm");
}

void FrictionLearner::determineFrictionCoefficient2(int start, int end)
{
  // Overall rolling direction (start to end) and rolled distance:
  Vector2f dir = balls.back().pos - balls.front().pos;
  dir.normalize();

  // Starting point in time and position
  const unsigned int t0 = balls.front().time;
  const Vector2f p0 = balls.front().pos;

  // Model:
  //   s(t) = s0 + v0 * t + 0.5 * a * t^2
  //
  // x(0) = s0     [m]
  // x(1) = v0     [m/s]
  // x(2) = a      [m/s^2]
  const int n = end - start + 1;
  Eigen::MatrixX3f A(n, 3);
  Eigen::VectorXf z(n);

  for(int i = start; i <= end; ++i)
  {
    const float t = (balls[i].time - t0) / 1000.f; // s

    // Position relative to starting point (in m)
    const Vector2f dp = (balls[i].pos - p0) / 1000.f;

    // Projection on rolling direction:
    const float s = dp.dot(dir);

    A(i - start, 0) = 1.f;
    A(i - start, 1) = t;
    A(i - start, 2) = 0.5f * t * t;
    z(i - start) = s;
  }

  // Determine result by Least Squares Fit:
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(z);
  const float s0 = x(0);       // m
  const float v0 = x(1);       // m/s
  const float a  = x(2);        // m/s^2

  Eigen::VectorXf residuals = A * x - z;
  const int dof = n - 3;
  const float sigma2 = residuals.squaredNorm() / static_cast<float>(dof);
  const float sigma = std::sqrt(sigma2); // m
  const Eigen::Matrix3f AtA = A.transpose() * A;
  const Eigen::Matrix3f cov = sigma2 * AtA.inverse();

  const float stdS0 = std::sqrt(std::max(cov(0, 0), 0.f)); // m
  const float stdV0 = std::sqrt(std::max(cov(1, 1), 0.f)); // m/s
  const float stdA  = std::sqrt(std::max(cov(2, 2), 0.f)); // m/s^2

  OUTPUT_TEXT("Result:\n"
              << "  s0=" << s0 << " m\n"
              << "  v0=" << v0 << " m/s\n"
              << "  a=" << a << " m/s^2");

  OUTPUT_TEXT("Standard deviations:\n"
              << "  stdS0=" << stdS0 * 1000.f << " mm\n"
              << "  stdV0=" << stdV0 << " m/s\n"
              << "  stdA=" << stdA << " m/s^2\n"
              << "  sigma=" << sigma * 1000.f << " mm");
}

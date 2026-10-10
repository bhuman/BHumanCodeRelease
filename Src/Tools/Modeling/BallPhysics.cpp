/**
 * @file BallPhysics.cpp
 *
 * Some functions for computations that involve the motion of a rolling ball.
 *
 * All functions assume a linear model for ball deceleration:
 *
 *   s = v * t + 0.5 * a * t^2  with v = ball velocity, a = ball friction, s = rolled distance
 *
 * @author Tim Laue
 */

#include "BallPhysics.h"
#include "Math/Eigen.h"
#include "Platform/BHAssert.h"
#include <limits>

Vector2f BallPhysics::getEndPosition(const Vector2f& p, const Vector2f& v, const BallSpecification& ballSpecs)
{
  const float tStop = computeTimeUntilBallStops(v, ballSpecs);  // unit: seconds
  return propagateBallPosition(p, v, tStop, ballSpecs);
}

Vector2f BallPhysics::propagateBallPosition(const Vector2f& p, const Vector2f& v, float t, const BallSpecification& ballSpecs)
{
  if(v.norm() == 0.f)
    return p;
  const float tStop = computeTimeUntilBallStops(v, ballSpecs);        // unit: seconds
  if(tStop < t)
    t = tStop;
  const Vector2f a = computeNegativeAccelerationVector(v, ballSpecs); // unit: millimeter / second^2
  return p + v * t + a * 0.5f * t * t;                                // unit: millimeter
}

void BallPhysics::propagateBallPositionAndVelocity(Vector2f& p, Vector2f& v, float t, const BallSpecification& ballSpecs)
{
  if(v.norm() == 0.f)
    return;
  const float tStop = computeTimeUntilBallStops(v, ballSpecs);           // unit: seconds
  if(tStop < t)
    t = tStop;
  const Vector2f a = computeNegativeAccelerationVector(v, ballSpecs);    // unit: millimeter / second^2
  p += v * t + a * 0.5f * t * t;                                         // unit: millimeter
  if(t == tStop)
    v = Vector2f::Zero();                                                // unit: millimeter / s
  else
    v += a * t;                                                          // unit: millimeter / s
}

void BallPhysics::applyFrictionToPositionAndVelocity(Vector2f& p, Vector2f& v, float t, const BallSpecification& ballSpecs)
{
  if(v.norm() == 0.f)
    return;
  const float tStop = computeTimeUntilBallStops(v, ballSpecs);           // unit: seconds
  if(tStop < t)
    t = tStop;
  const Vector2f a = computeNegativeAccelerationVector(v, ballSpecs);    // unit: millimeter / second^2
  if(t == tStop)
    v = Vector2f::Zero();                                                // unit: millimeter / s
  else
    v += a * t;                                                          // unit: millimeter / s
  p += a * t * t * 0.5f;                                                 // unit: millimeter
}

float BallPhysics::timeForDistance(const Vector2f& v, float distance, const BallSpecification& ballSpecs)
{
  if(getEndPosition(Vector2f::Zero(), v, ballSpecs).squaredNorm() < sqr(distance))
  {
    return std::numeric_limits<float>::max();
  }
  else
  {
    // Compute time by solving the standard equation:
    // s = v * t + 0.5 * a * t^2  with v = velocity, a = ballSpecs.staticFriction, s = distance
    const float s = distance / 1000.f;                       // unit: meter
    const Vector2f velmps = v / 1000.f;                      // unit: meter / second
    const float vb = velmps.norm() / ballSpecs.staticFriction;           // unit: seconds
    const float radicand = vb * vb + 2.f * s / ballSpecs.staticFriction; // unit: seconds^2
    if(radicand < 0.f)
      return std::numeric_limits<float>::max();
    else
      return (-std::sqrt(radicand) - vb);                    // unit: seconds
  }
}

float BallPhysics::velocityForDistance(const float distance, const BallSpecification& ballSpecs)
{
  ASSERT(distance > 0.0f);
  const float sqrt2 = 1.4142135623f;       // sqrt(2)
  const float b = ballSpecs.staticFriction * 1000.0f;  // unit: millimeter / second^2
  return sqrt2 * std::sqrt(-b * distance); // unit: millimeter / s
}

Vector2f BallPhysics::velocityAfterDistanceForTime(const Vector2f& p0, const Vector2f& p1, float deltaTime, const BallSpecification& ballSpecs)
{
  // The scalar velocity that the ball has now when friction is assumed for the rolling period (smaller than (p1-p0)/dt because ballSpecs.staticFriction is negative).
  const float velocityNow = (p1 - p0).norm() / deltaTime + 0.5f * 1000.f * ballSpecs.staticFriction * deltaTime;
  if(velocityNow <= 0.f)
    return Vector2f::Zero();
  return (p1 - p0).normalized(velocityNow);
}

Vector2f BallPhysics::computeNegativeAccelerationVector(const Vector2f& v, const BallSpecification& ballSpecs)
{
  Vector2f negVel(v * -1.f);                   // unit: millimeter / second
  negVel.normalize(std::abs(ballSpecs.staticFriction));    // unit: meter / second^2
  negVel *= 1000.f;                            // unit: millimeter / second^2
  return negVel;
}

float BallPhysics::computeTimeUntilBallStops(const Vector2f& v, const BallSpecification& ballSpecs)
{
  const Vector2f velInMetersPerSecond = v / 1000.f;            // unit: meter / second
  return (velInMetersPerSecond.norm() * -1.f) / ballSpecs.staticFriction;  // unit: seconds
}

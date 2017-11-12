/**
 * @file BallPhysics.h
 *
 * Some functions that help computations that involve the motion of a rolling ball.
 *
 * All functions assume a linear model for ball deceleration:
 *
 *   s = v * t + 0.5 * a * t^2  with v = ball velocity, a = ball friction, s = rolled distance
 *
 * @author <A href="mailto:tlaue@uni-bremen.de">Tim Laue</A>
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Math/Geometry.h"
#include <limits>

/**
 * @class BallPhysics
 *
 * Collection of static functions
 */
class BallPhysics
{
public:
  /**
   * Computes the position where a rolling ball is expected to stop rolling.
   * @param p The ball position (in mm)
   * @param v The ball velocity (in mm/s)
   * @param ballFriction The ball friction (negative force) (in m/s^2)
   * @return The position relative to the robot (in mm)
   */
  static Vector2f getEndPosition(const Vector2f& p, const Vector2f& v, float ballFriction)
  {
    ASSERT(ballFriction < 0.f);
    const float tStop = computeTimeUntilBallStops(v, ballFriction);  // unit: seconds
    return propagateBallPosition(p, v, tStop, ballFriction);
  }

  /**
   * Computes the position where a rolling ball is expected to stop rolling regarding the given rotation.
   * @param p The ball position (in mm)
   * @param v The ball velocity (in mm/s)
   * @param rotation The ball rotation
   * @param ballFriction The ball friction (negative force) (in m/s^2)
   * @return The position relative to the robot (in mm)
   */
  static Vector2f getEndPositionRegardingRotation(const Vector2f& p, const Vector2f& v, const float rotation, float ballFriction)
  {
    ASSERT(ballFriction < 0.f);
    const float tStop = computeTimeUntilBallStops(v, ballFriction);  // unit: seconds
    return propagateBallPositionWithRotation(p, v, tStop, ballFriction, rotation);
  }

  /**
   * Computes the position of a rolling ball in t seconds.
   * @param p The ball position (in mm)
   * @param v The ball velocity (in mm/s)
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The new position (in mm)
   */
  static Vector2f propagateBallPosition(const Vector2f& p, const Vector2f& v, float t, float ballFriction)
  {
    ASSERT(ballFriction < 0.f);
    if(v.norm() == 0.f)
      return p;
    const float tStop = computeTimeUntilBallStops(v, ballFriction);        // unit: seconds
    if(tStop < t)
      t = tStop;
    const Vector2f a = computeNegativeAccelerationVector(v, ballFriction); // unit: millimeter / second^2
    return p + v * t + a * 0.5f * t * t;                                   // unit: millimeter
  }

  /**
   * Computes the position and velocity of a rolling ball in t seconds regarding an observed rotation/spin without propagating the values.
   * The idea of this calculation is that the values are in their final position and we want the end position for the current state..
   * @param p The ball position (in mm)   -> updated by this method
   * @param v The ball velocity (in mm/s) -> updated by this method
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @rotation The rotation that will be applied on the movement (in radian)
   * @return The new position (in mm)
   */
  static Vector2f propagateBallPositionWithRotation(const Vector2f& p, const Vector2f& v, float t, float ballFriction, const float rotation)
  {
    ASSERT(ballFriction < 0.f);
    float velLength = v.norm();
    if(velLength == 0.f)
      return p;
    const float tStop = computeTimeUntilBallStops(v, ballFriction);        // unit: seconds
    if(tStop < t)
      t = tStop;
    if(rotation == 0.f)
      return propagateBallPosition(p, v, t, ballFriction);
    const Vector2f a = computeNegativeAccelerationVector(v, ballFriction); // unit: millimeter / second^2
    const Matrix2f rot = Eigen::Rotation2D<float>(rotation * t).toRotationMatrix();
    return p + rot * (v * t + a * 0.5f * t * t);                                          // unit: millimeter
  }

  /**
   * Computes the position and velocity of a rolling ball in t seconds.
   * @param p The ball position (in mm)   -> updated by this method
   * @param v The ball velocity (in mm/s) -> updated by this method
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   */
  static void propagateBallPositionAndVelocity(Vector2f& p, Vector2f& v, float t, float ballFriction)
  {
    ASSERT(ballFriction < 0.f);
    if(v.norm() == 0.f)
      return;
    const float tStop = computeTimeUntilBallStops(v, ballFriction);        // unit: seconds
    if(tStop < t)
      t = tStop;
    const Vector2f a = computeNegativeAccelerationVector(v, ballFriction); // unit: millimeter / second^2
    p += v * t + a * 0.5f * t * t;                                         // unit: millimeter
    if(t == tStop)
      v = Vector2f::Zero();                                                // unit: millimeter / s
    else
      v += a * t;                                                          // unit: millimeter / s
  }

  /**
   * Computes the position and velocity of a rolling ball in t seconds regarding an observed rotation/spin.
   * @param p The ball position (in mm)   -> updated by this method
   * @param v The ball velocity (in mm/s) -> updated by this method
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @rotation The rotation that will be applied on the movement (in radian)
   */
  static void propagateBallPositionAndVelocityWithRotation(Vector2f& p, Vector2f& v, float t, float ballFriction, float& rotation)
  {
    ASSERT(ballFriction < 0.f);
    float velLength = v.norm();
    if(velLength == 0.f)
      return;
    const float tStop = computeTimeUntilBallStops(v, ballFriction);        // unit: seconds
    if(tStop < t)
      t = tStop;
    if(rotation == 0.f)
      return propagateBallPositionAndVelocity(p, v, t, ballFriction);
    const Vector2f a = computeNegativeAccelerationVector(v, ballFriction); // unit: millimeter / second^2
    const Matrix2f rot = Eigen::Rotation2D<float>(rotation * t).toRotationMatrix();
    p += rot * (v * t + a * 0.5f * t * t);                                          // unit: millimeter
    v += a * t;     // unit: millimeter / s
    //Please don't change the code due to debug reasons, sincerely Alex
    float stabilityValue = 1000.f / velLength;
    float minResult = std::min(1.5f, stabilityValue);
    float maxResult = std::max(minResult, 1.f);
    rotation *= maxResult;
  }

  /**
   * Applies friction to position and velocity of a rolling ball in t seconds.
   * THIS METHOD DOES NOT APPLY THE VELOCITY TO THE POSITION! USE propagateBallPositionAndVelocity INSTEAD!
   * (this might appear freaky but fits the structure of the current BallLocator;
   *  the purpose of this function is to have all code that makes an assumption about
   *  the current model for rolling friction in one file)
   * @paarm p The ball position (in mm), set by this method
   * @param v The ball velocity (in mm/s), set by this method
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The new velocity (in mm / s)
   */
  static void applyFrictionToPositionAndVelocity(Vector2f& p, Vector2f& v, float t, float ballFriction)
  {
    ASSERT(ballFriction < 0.f);
    if(v.norm() == 0.f)
      return;
    const float tStop = computeTimeUntilBallStops(v, ballFriction);        // unit: seconds
    if(tStop < t)
      t = tStop;
    const Vector2f a = computeNegativeAccelerationVector(v, ballFriction); // unit: millimeter / second^2
    if(t == tStop)
      v = Vector2f::Zero();                                                // unit: millimeter / s
    else
      v += a * t;                                                          // unit: millimeter / s
    p += a * t * t * 0.5f;                                                 // unit: millimeter
  }

  /**
   * Computes the time (in seconds) the ball needs to pass distance.
   * @param v The ball velocity (in mm/s)
   * @distance The distance (in mm)
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return std::numeric_limits<float>::max(), if ball won't make the distance with its current velocity, time otherwise
   */
  static float timeForDistance(const Vector2f& v, float distance, float ballFriction)
  {
    ASSERT(ballFriction < 0.f);
    if(getEndPosition(Vector2f::Zero(), v, ballFriction).norm() < distance)
    {
      return std::numeric_limits<float>::max();
    }
    else
    {
      // Compute time by solving the standard equation:
      // s = v * t + 0.5 * a * t^2  with v = velocity, a = ballFriction, s = distance
      const float s = distance / 1000.f;                       // unit: meter
      const Vector2f velmps = v / 1000.f;                      // unit: meter / second
      const float vb = velmps.norm() / ballFriction;           // unit: seconds
      const float radicand = vb * vb + 2.f * s / ballFriction; // unit: seconds^2
      if(radicand < 0.f)
        return std::numeric_limits<float>::max();
      else
        return (-std::sqrt(radicand) - vb);                    // unit: seconds
    }
  }

  /**
   * Calculates the velocity needed to kick the ball a certain distance
   * @param distance the distance in mm
   * @param ballFriction The ball friction (negative force)  (in m/s^2)
   * @return the velocity in mm/s
   */
  static float velocityForDistance(const float distance, const float ballFriction)
  {
    ASSERT(ballFriction < 0.0f);
    ASSERT(distance > 0.0f);
    const float sqrt2 = 1.4142135623f;       // sqrt(2)
    const float b = ballFriction * 1000.0f;  // unit: millimeter / second^2
    return sqrt2 * std::sqrt(-b * distance); // unit: millimeter / s
  }

  /**
   * A function that propagates the ball state(position, velocity, rotation) assuming
   * there is a collision with a robot.
   * @param robotPosition, the pose of the robot
   * @param position, the position of the ball
   * @param velocity, the velocity of the ball
   * @param rotation, the rotation of the ball
   * @param ballFrictiom, the friction
   */
  static void computeBallCollisionWithRobot(const Pose2f& robotPosition, Vector2f& position, Vector2f& velocity,
      float rotation, const float ballFriction)
  {
    ASSERT(ballFriction < 0.0f);
    Vector2f positionDifference = robotPosition.translation - position;
    Vector2f oldPosition = position;
    float distance = positionDifference.norm();
    Vector2f startVelocity = velocity;
    float neededTime = timeForDistance(startVelocity, distance, ballFriction);
    if(neededTime == std::numeric_limits<float>::max())
      return;
    neededTime *= (float) 0.98; // We want to be just a small step ahead
    propagateBallPositionAndVelocityWithRotation(position, velocity, neededTime, ballFriction, rotation);
    ////alextastic collision
    Vector2f reflectionVelocity = velocity, backupVelocity = velocity;
    if(!Geometry::getCollisionReflectionWithRectangleRobotFeet(robotPosition, position, velocity, reflectionVelocity))
    {
      float tStop = computeTimeUntilBallStops(velocity, ballFriction);
      propagateBallPositionAndVelocityWithRotation(position, velocity, tStop, ballFriction, rotation);
      return;
    }
    const float accelerationFactor = 0.75;
    velocity = reflectionVelocity.normalized() * backupVelocity.norm() * accelerationFactor;
    Angle angle = Geometry::angleTo(robotPosition, velocity);
    //Is there a chance that the ball gets his kind of random spin
    int rotationSign = (0.f < angle) - (angle < 0.f);
    if(velocity.squaredNorm() < 1000.f)
      rotation = 0.349f * rotationSign; //20 degree in the other direction
    else
      rotation = 0.f;
    float ballStopTime = computeTimeUntilBallStops(velocity, ballFriction);
    propagateBallPositionAndVelocityWithRotation(position, velocity, ballStopTime, ballFriction, rotation);
  }

private:
  /**
   * Computes a vector describing the ball acceleration (in reverse direction of the velocity)
   * @param v The ball velocity (in mm/s)
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The acceleration vector (in millimeter / second^2)
   */
  static Vector2f computeNegativeAccelerationVector(const Vector2f& v, float ballFriction)
  {
    Vector2f negVel(v * -1.f);                   // unit: millimeter / second
    negVel.normalize(std::abs(ballFriction));    // unit: meter / second^2
    negVel *= 1000.f;                            // unit: millimeter / second^2
    return negVel;
  }

  /**
   * Computes the remaining time until the ball comes to a stop
   * @param v The ball velocity (in mm/s)
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The remaining rolling time (seconds)
   */
  static float computeTimeUntilBallStops(const Vector2f& v, float ballFriction)
  {
    const Vector2f velInMetersPerSecond = v / 1000.f;            // unit: meter / second
    return (velInMetersPerSecond.norm() * -1.f) / ballFriction;  // unit: seconds
  }
};

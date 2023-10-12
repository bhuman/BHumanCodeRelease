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

#include "Math/Eigen.h"

/**
 * @class BallPhysics
 *
 * Collection of static functions
 */
struct BallPhysics
{
  /**
   * Computes the position where a rolling ball is expected to stop rolling.
   * @param p The ball position (in mm)
   * @param v The ball velocity (in mm/s)
   * @param ballFriction The ball friction (negative force) (in m/s^2)
   * @return The position relative to the robot (in mm)
   */
  static Vector2f getEndPosition(const Vector2f& p, const Vector2f& v, float ballFriction);

  /**
   * Computes the position of a rolling ball in t seconds.
   * @param p The ball position (in mm)
   * @param v The ball velocity (in mm/s)
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The new position (in mm)
   */
  static Vector2f propagateBallPosition(const Vector2f& p, const Vector2f& v, float t, float ballFriction);

  /**
   * Computes the position and velocity of a rolling ball in t seconds.
   * @param p The ball position (in mm)   -> updated by this method
   * @param v The ball velocity (in mm/s) -> updated by this method
   * @t Time in seconds
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   */
  static void propagateBallPositionAndVelocity(Vector2f& p, Vector2f& v, float t, float ballFriction);

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
  static void applyFrictionToPositionAndVelocity(Vector2f& p, Vector2f& v, float t, float ballFriction);

  /**
   * Computes the time (in seconds) the ball needs to pass distance.
   * @param v The ball velocity (in mm/s)
   * @distance The distance (in mm)
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return std::numeric_limits<float>::max(), if ball won't make the distance with its current velocity, time otherwise
   */
  static float timeForDistance(const Vector2f& v, float distance, float ballFriction);

  /**
   * Calculates the velocity needed to kick the ball a certain distance
   * @param distance the distance in mm
   * @param ballFriction The ball friction (negative force)  (in m/s^2)
   * @return the velocity in mm/s
   */
  static float velocityForDistance(const float distance, const float ballFriction);

  /**
   * Computes the velocity that the ball has after needing a specific time for moving from one point to another
   * @param p0 The previous point of the ball trajectory
   * @param p1 The current point of the ball trajectory
   * @param deltaTime The time it took the ball to travel from p0 to p1 (seconds)
   * @param ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The velocity the ball has now (at p1).
   */
  static Vector2f velocityAfterDistanceForTime(const Vector2f& p0, const Vector2f& p1, float deltaTime, float ballFriction);

  /**
   * Computes a vector describing the ball acceleration (in reverse direction of the velocity)
   * @param v The ball velocity (in mm/s)
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The acceleration vector (in millimeter / second^2)
   */
  static Vector2f computeNegativeAccelerationVector(const Vector2f& v, float ballFriction);

  /**
   * Computes the remaining time until the ball comes to a stop
   * @param v The ball velocity (in mm/s)
   * @ballFriction The ball friction (negative force)  (in m/s^2)
   * @return The remaining rolling time (seconds)
   */
  static float computeTimeUntilBallStops(const Vector2f& v, float ballFriction);
};

/**
 * File:   PT2.h
 * Author: arne
 *
 * Created on October 31, 2014, 3:26 PM
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

/**
 * A second order linear time invariant system with velocity limit.
 * Called "PT2" in German (see http://de.wikipedia.org/wiki/PT2-Glied)
 */
STREAMABLE(PT2,
{
  PT2() = default;

  /**
   * @param T time constant (wiki: Physically, the constant represents the time it takes the system's
   *                                   step response to reach  \approx 63.2% of its final (asymptotic)
   *                                   value for systems that increase in value (say from a step increase),
   *                                   or it can represent the time for systems to decrease in value by a
   *                                   \approx 36.8% factor (say from a step decrease))
   * @param D dampening factor. The lower D is, the more the system will oscillate
   * @param K amplification constant. Defines how good the steady state will match the targeted state.
   *          I.e. if the target state is 100 and K is 0.1 the steady state will be 10.
   *          If K is 1.1 the steady state will be 110.
   *          This should usually be a value to 1.0
   * @param initialPosition Current position in rad
   * @param initialVelocity Current velocity in rad/s
   * @param V Maximum allowed velocity in rad/s
   */
  PT2(float T, float D, float K, float initialPosition,
      float initialVelocity, float maxVelocity);

  void initialize(const float initialPosition, const float initialVelocity);

  /**
   * Iterates the model. I.e. moves the model one step closer to the specified
   * goal position.
   * @param goal The position the motor should drive to
   * @param dt Time between steps in s
   */
  void step(const float goal, const float dt);

  /**Returns the current motor position */
  float getPosition() const;

  /**Returns the current motor velocity in rad/s */
  float getVelocity() const;

  /**Returns the acceleration that was used to calculate the current velocity in rad/s */
  float getAcceleration() const;

  bool isInitialized() const;
  /**Returns true if the parameters are valid, false otherwise*/
  bool isValid() const;

  float x = 0.f; /*< Intermediate value, can be related to current velocity by: x / T */
  float y = 0.f;/*< Current position in rad */
  float yd = 0.f;/*< current velocity in rad/s */
  float ydd = 0.f;/*< current acceleration in rad/s */
  bool initialized = false, /**<If initialize has been called */

  // See constructor for variable description
  (float)(0.f) T,
  (float)(0.f) D,
  (float)(0.f) K,
  (float)(0.f) V,
});

/*
 * File:   CanonicalSystem.h
 * Author: arne
 *
 * Created on January 28, 2015, 1:10 PM
 */

#pragma once

/**
 * A canonical dynamical system can be used to replace the time in dynamical systems
 * with an exponentially falling phase value.
 * This canonical system implements the following differential equation:
 * \f[
 *    T\dot{z} = -\alpha z
 * \f]
 * Where T is a temporal scaling factor, z is the phase value between 1.0 and 0.0
 * and alpha is a decay factor that defines how fast the exponential decay is.
 */
class CanonicalSystem
{
public:
  float z = 1.f; /**<Current phase */
  float currentTime = 0.f; /**<The time corresponding to the current phase */

private:
  float T = 0.f; /**<Temporal scaling factor (executionTime) */
  float alpha = 0.f; /**<decay factor */

public:
  CanonicalSystem() = default;
  CanonicalSystem(const float executionTime, const float startingPhase = 1.0f,
                  const float finalPhaseValue = 0.01f);

  /**
   * Iterates the dynamical system and returns the next phase value.
   */
  float step(const float dt);

  /**
   * Resets the canonical system to the beginning
   */
  void reset();

  /**
   * Returns the phase at the given time.
   * @note This method is only intended for initialization. Use step() for
   *       iteration.
   */
  float getPhaseAt(const float time) const;
};

/**
 * @file Math/Random.h
 * This contains some functions for creating random numbers.
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include <cstdlib>
#include <random>
#include <chrono>

static std::default_random_engine randomGenerator(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));

/**
 * The function returns a random number in the range of [0..1].
 * @return The random number.
 */
static inline float randomFloat()
{
  std::uniform_real_distribution<float> rnd(0.0, 1.0);
  return rnd(randomGenerator);
}

static inline float randomFloat(float min, float max)
{
  std::uniform_real_distribution<float> rnd(min, max);
  return rnd(randomGenerator);
}

/**
 * The function returns a random integer number in the range of [0..n-1].
 * @param n the number of possible return values (0 ... n-1)
 * @return The random number.
 */
template<typename T, typename = std::enable_if<std::is_integral<T>::value>>
static inline T random(T n)
{
  std::uniform_int_distribution<T> rnd(0, n - 1);
  return rnd(randomGenerator);
}

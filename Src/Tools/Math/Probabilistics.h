/**
 * @file Math/Probabilistics.h
 *
 * This contains some probabilistic functions
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Common.h"
#include "Random.h"
#include <algorithm>


/** constant for e*/
const float e = 2.71828182845902353602874713527f;

/** constant for triangular distribution*/
const float sqrt6dividedBy2 = std::sqrt(6.0f) / 2.0f;

/**
* Returns a gaussian random deviate
* @return As mentioned above
*/
inline float randomGauss()
{
  float r, v1, v2;
  do
  {
    v1 = 2.0f * randomFloat() - 1.0f;
    v2 = 2.0f * randomFloat() - 1.0f;
    r = v1 * v1 + v2 * v2;
  }
  while(r >= 1.0f || r == 0);
  const float fac(std::sqrt(-2.0f * std::log(r) / r));
  return v1 * fac;
}

/**
* Returns the probability of a value in a gaussian distribution
* @param x The value
* @param s The standard deviation
* @return The probability density at x
*/
inline float gaussianProbability(float x, float s)
{
  return std::max(1.0f / (s * sqrt2pi) * std::exp(-0.5f * sqr(x / s)), 0.000001f);
}

/**
* Sampling from approximated normal distribution with zero mean and
* standard deviation b. C.f. "Probabilistic Robotics", Table 5.4
* @param b The standard deviation
* @return The sampled value
*/
inline float sampleNormalDistribution(float b)
{
  float result(0.0f);
  for(int i = 0; i < 12; i++)
    result += 2.0f * ((randomFloat() - 0.5f) * b);
  return result / 2.0f;
}

/**
* Sampling from approximated normal distribution with zero mean and
* standard deviation b. C.f. "Probabilistic Robotics", Table 5.4
* This is an integer version which uses randomFast, so use it only for large n
* @param b The standard deviation
* @return The sampled value
*/
inline int sampleNormalDistribution(int b)
{
  if(b != 0)
  {
    int result(0);
    for(int i = 0; i < 12; i++)
      result += random(2 * b) - b;
    return result / 2;
  }
  else
    return 0;
}

/**
* Sampling from a triangular distribution with zero mean and
* standard deviation b. C.f. "Probabilistic Robotics", Table 5.4
* @param b The standard deviation
* @return The sampled value
*/
inline float sampleTriangularDistribution(float b)
{
  float randResult = 2.0f * ((randomFloat() - 0.5f) * b) + 2.0f * ((randomFloat() - 0.5f) * b);
  return sqrt6dividedBy2 * randResult;
}

/**
* Sampling from a triangular distribution with zero mean and
* standard deviation b. C.f. "Probabilistic Robotics", Table 5.4
* This is an integer version which uses randomFast, so use it only for large n
* @param b The standard deviation
* @return The sampled value
*/
inline int sampleTriangularDistribution(int b)
{
  if(b != 0)
  {
    int randResult = (random(2 * b) - b) + (random(2 * b) - b);
    return static_cast<int>(sqrt6dividedBy2 * randResult);
  }
  else
    return 0;
}

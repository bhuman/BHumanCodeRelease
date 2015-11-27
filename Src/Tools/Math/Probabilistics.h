/**
 * @file Math/Probabilistics.h
 *
 * This contains some probabilistic functions
 *
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "BHMath.h"
#include "Random.h"
#include <cmath>
#include <algorithm>

/** constant for an expression used by the gaussian function*/
const float sqrt2pi = std::sqrt(2.0f * pi);

/** constant for triangular distribution*/
const float sqrt6dividedBy2 = std::sqrt(6.0f) / 2.0f;

/** constant for sqrt(2) */
const float sqrt2 = std::sqrt(2.f);

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

/**
* The probability that a normal distributed random variable X(mu,s)
*  will take a value inside the interval (a,b), where a < b
* @param mu The mean of the normal distribution
* @param s  The standard deviation of the normal distribution
* @param a  The lower bound of the interval
* @param b  The upper bound of the interval
*/
inline float probabiltyOfInterval(float mu, float s, float a, float b)
{
  const float pa = std::erf((a - mu) / (sqrt2 * s));
  const float pb = std::erf((b - mu) / (sqrt2 * s));
  return (pb - pa) / 2;
}

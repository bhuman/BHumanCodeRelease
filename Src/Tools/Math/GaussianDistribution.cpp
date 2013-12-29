/**
* @file GaussianDistribution.cpp
*
* Implementation of class GaussianDistribution
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#include "Probabilistics.h"
#include "GaussianDistribution.h"


float GaussianDistribution::distanceTo(const GaussianDistribution& other) const
{
  float diff(mean - other.mean);
  float invVariance = 1.0f / variance;
  return diff * (invVariance * diff);
}

float GaussianDistribution::probabilityAt(float p) const
{
  float diff(p - mean);
  float invVariance = 1.0f / variance;
  float exponent(diff * invVariance * diff);
  float probability(1.0f / (std::sqrt(pi2) * std::sqrt(variance)));
  probability *= exp(-0.5f * exponent);
  return std::max(probability, 0.000001f);
}

void GaussianDistribution::generateDistributionFromMeasurements(float* x, int numOfX)
{
  if(numOfX < 2)
    return;
  mean = 0.0f;
  for(int i = 0; i < numOfX; i++)
    mean += x[i];
  mean /= numOfX;
  variance = 0.0f;
  for(int i = 0; i < numOfX; i++)
    variance += (x[i] - mean) * (x[i] - mean);
  variance *= 1.0f / (numOfX - 1);
}

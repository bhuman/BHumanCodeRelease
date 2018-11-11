/**
 * @file Differentiator.cpp
 * Implementation of the methods of a class which differentiates a signal.
 *
 * @author: Felix Wenk
 */

#include "Differentiator.h"
#include "Tools/Math/BHMath.h"

Differentiator::Differentiator()
: r(0.02f), rSqr(sqr(r)), stepsize(10.0f), prediction(0.0f), derivative(0.0f)
{}

Differentiator& Differentiator::operator=(const Differentiator& other)
{
  prediction = other.prediction;
  derivative = other.derivative;
  derivative2 = other.derivative2;

  return *this;
}

float Differentiator::update(const float input)
{
  /* Prediction from the last step is approx. the signal of this frame. */
  const float error = Angle::normalize(prediction - input);
  prediction += stepsize * derivative;
  prediction = Angle::normalize(prediction);
  derivative2 = -2.0f * rSqr * error - 2.0f * r * derivative;
  derivative += stepsize * derivative2;

  return derivative;
}

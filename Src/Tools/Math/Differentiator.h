/**
 * @file Differentiator.h
 * Declaration of a class that differentiates an angular signal.
 *
 * @author: Felix Wenk
 * @author: Jesse Richter-Klug (Only reintegration)
 */

#pragma once

class Differentiator
{
public:
  Differentiator();
  Differentiator& operator=(const Differentiator& other);
  /**
   * Update the differentiator with the next input measurement
   * and returns the next value of the derivative signal.
   */
  float update(const float input);
  const float r;
  const float rSqr;
  const float stepsize;
  /* Comments apply for the situation after calling update */
  float prediction; /**< Expected input in the next step. */
  float derivative; /**< Slope at the next step. */
  float derivative2; /**< Slope of the slope at this step. */
};

/**
 * @file Math/Probabilistics.h
 *
 * This contains some probabilistic functions
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "BHMath.h"
#include "Covariance.h"
#include "Eigen.h"
#include "Random.h"
#include <cmath>
#include <algorithm>

/** constant for an expression used by the gaussian function*/
const float sqrt2pi = std::sqrt(2.0f * pi);

/** constant for sqrt(2) */
const float sqrt2 = std::sqrt(2.f);

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

/**
 * This function solves the two dimensional square equation.
 * @param mean The mean of the normal distribution. This variable will be updated
 * @param covariance The covariance matrix of the normal distribution. This variable will be updated
 * @param mean2 The mean of the second normal distribution
 * @param covariance2 The covariance matrix of the second normal distribution
 * @return Whether the equation could be solved
 */
inline bool twoDimSquareEquation(Vector2f& mean, Matrix2f& covariance, const Vector2f& mean2, const Matrix2f& covariance2)
{
  //multidimensional square equation (german: "Multidimensionale quadratische Ausgleichsrechnung" aus dem Skript
  //"Theorie der Sensorfusion")
  const Eigen::Matrix<float, 4, 2> A = (Eigen::Matrix<float, 4, 2>() << Matrix2f::Identity(), Matrix2f::Identity()).finished();
  const Eigen::Matrix<float, 2, 4> AT = A.transpose();
  const Eigen::Matrix4f Sigma = ((Eigen::Matrix4f() << covariance, Matrix2f::Zero(), Matrix2f::Zero(), covariance2).finished());
  if(Sigma.determinant() == 0)
    return false;

  const Eigen::Matrix<float, 4, 4> SigmaInv = Sigma.inverse();
  const Eigen::Vector4f z = (Eigen::Vector4f() << mean, mean2).finished();
  covariance = ((AT * SigmaInv) * A).inverse();
  Covariance::fixCovariance(covariance);
  mean = covariance * AT * SigmaInv * z;
  ASSERT(mean.allFinite());
  ASSERT(covariance.allFinite());
  return true;
}

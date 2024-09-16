/**
 * @file BallLocatorTools.h
 * Some functions that might be used by different BallLocator implementations
 * @author Colin Graf
 * @author Tim Laue
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/BodyContour.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/ImagePreprocessing/ImageCoordinateSystem.h"
#include "Math/BHMath.h"
#include "Math/Geometry.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

class BallLocatorTools
{
public:
  /**
   * Returns the *unnormalized* negative log-likelihood of a 2D Gaussian, parameterized by mean and covariance matrix.
   * The log(2pi) + log(det(cov))/2 term is omitted because it is the same for all positions.
   * @param mean The mean of the Gaussian distribution.
   * @param cov The covariance matrix of the Gaussian distribution.
   * @param pos The position/measurement at which to evaluate the negative log-likelihood.
   * @return The unnormalized negative log-likelihood ...
   */
  static float getNLLOfPosition(const Vector2f& mean, const Matrix2f& cov, const Vector2f& pos)
  {
    const Vector2f diff = pos - mean;
    const float mahalanobisDistanceSqr = diff.dot(cov.inverse() * diff);
    return 0.5f * mahalanobisDistanceSqr;
  }

  /**
   * Returns the negative log-likelihood of a 2D Gaussian at its mean.
   * This omits the log(2pi) term because that is the same for all covariance matrices.
   * This means that this function should only be used if a common offset does not matter.
   * @param cov The covariance matrix of the Gaussian.
   * @return The negative log-likelihood ...
   */
  static float getNLLOfMean(const Matrix2f& cov)
  {
    return 0.5f * std::log(std::max(cov.determinant(), 0.f));
  }

  /**
   * Function calculates possible intersection between a line and a circle.
   * @param lineBase the base of the line
   * @param lineDir the direction of the line
   * @param circleBase the center of the circle
   * @param circleRadius the radius of the circle
   * @param factor the factor to stretch the radius
   * @return true, if there is a possible intersection, else false
   */
  static bool getSmallestLineWithCircleIntersectionFactor(const Vector2f& lineBase, const Vector2f& lineDir, const Vector2f& circleBase, float circleRadius, float& factor)
  {
    const float a = lineDir.dot(lineDir);
    if(a == 0.f) // Check if the direction is zero vector
      return false;
    const Vector2f base = lineBase - circleBase;
    const float b_2 = lineDir.dot(base);
    const float c = base.dot(base) - sqr(circleRadius);
    const float ll_4 = b_2 * b_2 - a * c;
    if(ll_4 < 0.f)
      return false;
    if(a > 0)
      factor = (-b_2 - std::sqrt(ll_4)) / a;
    else
      factor = (-b_2 + std::sqrt(ll_4)) / a;
    return true;
  }

  static bool getLineWithLineIntersectionFactors(const Vector2f& lineBase1, const Vector2f& lineDir1, const Vector2f& lineBase2, const Vector2f& lineDir2, float& factor1, float& factor2)
  {
    const float h = lineDir1.x() * lineDir2.y() - lineDir1.y() * lineDir2.x();
    if(h == 0.f)
      return false;
    factor2 = ((lineBase2.x() - lineBase1.x()) * lineDir1.y() - (lineBase2.y() - lineBase1.y()) * lineDir1.x()) / h;
    factor1 = ((lineBase1.y() - lineBase2.y()) * lineDir2.x() - (lineBase1.x() - lineBase2.x()) * lineDir2.y()) / h;
    return true;
  }

  /**
   * Checks if a ball is within the robot's current field of view and should thus be detected by the ball perceptor
   * @param expectedBallPositionOnField A ball position in coordinates relative to the robot
   * @param expectedBallRadiusOnField The ball radius "in reality"
   * @param theCameraMatrix Information about the current perspective
   * @param theCameraInfo Information about the current image
   * @param theBodyContour Information about the projection of the robot's body into the image
   * @param theImageCoordinateSystem Well... the ImageCoordinateSystem?!
   * @param ballRadiusInImageThresholdModifiers If not specified, only balls that are completely inside the image are checked. By setting a value 0 < x < 1, balls closer to the image border are considered, too.
   * @return true, if the ball is located within the robot's current field of view
   */
  static bool ballShouldBeVisibleInCurrentImage(const Vector2f& expectedBallPositionOnField, float expectedBallRadiusOnField,
                                                const CameraMatrix& theCameraMatrix, const CameraInfo& theCameraInfo,
                                                const BodyContour& theBodyContour, const ImageCoordinateSystem& theImageCoordinateSystem,
                                                float ballRadiusInImageThresholdModifier = 1.f)
  {
    if(!theCameraMatrix.isValid)
      return false;
    Vector3f ballRel;
    ballRel << expectedBallPositionOnField, expectedBallRadiusOnField;
    Vector2f ballInImage;
    if(!Transformation::robotToImage(ballRel, theCameraMatrix, theCameraInfo, ballInImage))
      return false;
    ballInImage = theImageCoordinateSystem.fromCorrected(ballInImage);
    const float ballDistance = (theCameraMatrix.inverse() * ballRel).norm();
    const float ballRadiusInImage = Projection::getSizeByDistance(theCameraInfo, expectedBallRadiusOnField, ballDistance);
    const float offset = std::ceil(ballRadiusInImage) * ballRadiusInImageThresholdModifier;
    if(ballInImage.x() < offset || ballInImage.x() >= theCameraInfo.width - offset || ballInImage.y() < offset || ballInImage.y() >= theCameraInfo.height - offset)
      return false;
    if(!theBodyContour.isValidPoint(ballInImage.cast<int>()))
      return false;
    return true;
  }
};

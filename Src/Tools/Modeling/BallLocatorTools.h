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
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Projection.h"
#include "Tools/Math/Transformation.h"

class BallLocatorTools
{
public:
  static float getLikelihoodOfPosition(const Vector2f& mean, const Matrix2f& cov, const Vector2f& pos)
  {
    Vector2f diff = pos - mean;
    float exponent = diff.dot(cov.inverse() * diff);
    float p = std::exp(-0.5f * exponent);
    return std::max(p, 0.01f /*0.0000001f*/);
  }

  static float getLikelihoodOfMean(const Matrix2f& cov)
  {
    return 1.f / std::max((pi2 * std::sqrt(std::max(cov.determinant(), 0.f))), 0.0000001f);
  }

  /**
   * Function calculates possible intersection between a line and a circle.
   * @param lineBase, the base of the line
   * @param lineDir, the direction of the line
   * @param circleBase, the center of the circle
   * @param circleRadius, the radius of the circle
   * @param factor, the factor to stretch the radius
   * @return true, if there is a possible intersection, else false
   */
  static bool getSmallestLineWithCircleIntersectionFactor(const Vector2f& lineBase, const Vector2f& lineDir, const Vector2f& circleBase, float circleRadius, float& factor)
  {
    const Vector2f& dir = lineDir;
    float a = lineDir.dot(lineDir);
    if(a == 0.f) // Check if the dirction is zero vector
      return false;
    const Vector2f base = lineBase - circleBase;
    float b = 2.f * (dir.dot(base));
    float c = base.dot(base) - sqr(circleRadius);
    float ll = b * b - 4.f * a * c;
    if(ll < 0.f)
      return false;
    if(a > 0)
      factor = (-b - std::sqrt(ll)) / (2.f * a);
    else
      factor = (-b + std::sqrt(ll)) / (2.f * a);
    return true;
  }

  static bool getLineWithLineIntersectionFactors(const Vector2f& lineBase1, const Vector2f& lineDir1, const Vector2f& lineBase2, const Vector2f& lineDir2, float& factor1, float& factor2)
  {
    float h = lineDir1.x() * lineDir2.y() - lineDir1.y() * lineDir2.x();
    if(h == 0.f)
      return false;
    factor2 = ((lineBase2.x() - lineBase1.x()) * lineDir1.y() - (lineBase2.y() - lineBase1.y()) * lineDir1.x()) / h;
    factor1 = ((lineBase1.y() - lineBase2.y()) * lineDir2.x() - (lineBase1.x() - lineBase2.x()) * lineDir2.y()) / h;
    return true;
  }

  /** Checks, if a ball is within the robot's current field of view and should thus be detected by the BallPerceptor
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

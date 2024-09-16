/**
 * @file LegacyMeasurementCovarianceProvider.cpp
 *
 * This module provides the function that was previously implemented in Measurements::positionToCovarianceMatrixInRobotCoordinates.
 * Although this approach for determining an uncertainty for a given perception has been used by B-Human for over a decade, it should
 * be replaced with a more sophisticated approach soon.
 * Having this functionality inside a module makes this replacement easier.
 *
 * @author Tim Laue
 */

#include "LegacyMeasurementCovarianceProvider.h"

MAKE_MODULE(LegacyMeasurementCovarianceProvider);

void LegacyMeasurementCovarianceProvider::update(MeasurementCovariance& measurementCovariance)
{
  inverseCameraMatrix = theCameraMatrix.inverse();
  currentDefaultRotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? robotRotationDeviationInStandDefault : robotRotationDeviationDefault;
  DEBUG_DRAWING("module:LegacyMeasurementCovarianceProvider", "drawingOnField")
  {
    draw(measurementCovariance);
  }

  measurementCovariance.computeForRelativePosition = [&](const Vector2f& p) -> Matrix2f
  {
    return computeCovariance(p, currentDefaultRotationDeviation);
  };

  measurementCovariance.transformWithCovLegacy = [&](const Vector2f& inImage, float z, const Vector2f& rotationDeviation, Vector2f& inRobot,
                                                     Matrix2f& covariance) -> bool
  {
    auto out = Transformation::imageToRobotHorizontalPlane(inImage, z, theCameraMatrix, theCameraInfo, inRobot);
    if(out)
      covariance = computeCovariance(inRobot, rotationDeviation);
    return out;
  };

  measurementCovariance.transformPointWithCov = [&](const Vector2f& inImage, float z, Vector2f& inRobot, Matrix2f& covariance) -> bool
  {
    return measurementCovariance.transformWithCovLegacy(inImage, z, currentDefaultRotationDeviation, inRobot, covariance);
  };
}

Matrix2f LegacyMeasurementCovarianceProvider::computeCovariance(const Vector2f& p, const Vector2f& rotationDeviation) const
{
  const float pointZInWorld = 0.f; // For allowing positions that are not on the floor, make this a parameter.
  const Vector3f unscaledVectorToPoint = inverseCameraMatrix * Vector3f(p.x(), p.y(), pointZInWorld);
  const Vector3f unscaledWorld = theCameraMatrix.rotation * unscaledVectorToPoint;
  const float h = theCameraMatrix.translation.z() - pointZInWorld;
  const float scale = h / -unscaledWorld.z();
  const Vector2f pointInWorld(unscaledWorld.x() * scale, unscaledWorld.y() * scale);
  const float distance = pointInWorld.norm();
  Matrix2f rot;
  if(distance == 0.f)
  {
    rot = Matrix2f::Identity();
  }
  else
  {
    const Vector2f cossin = pointInWorld * (1.f / distance);
    rot << cossin.x(), -cossin.y(),
           cossin.y(), cossin.x();
  }
  Matrix2f cov;
  cov << sqr(h / std::tan((distance == 0.f ? pi_2 : std::atan(h / distance)) - rotationDeviation.x()) - distance), 0.f,
         0.f, sqr(std::tan(rotationDeviation.y()) * distance);
  cov = rot * cov * rot.transpose();
  Covariance::fixCovariance<2>(cov);
  return cov;
}

void LegacyMeasurementCovarianceProvider::draw(const MeasurementCovariance& measurementCovariance)
{
  for(float d = 1000.f; d <= 5001.f; d += 1000.f)
  {
    const Vector2f p(d, 0.f);
    const Matrix2f cov = measurementCovariance.computeForRelativePosition(p);
    COVARIANCE_ELLIPSES_2D("module:LegacyMeasurementCovarianceProvider", cov, p);
  }
}

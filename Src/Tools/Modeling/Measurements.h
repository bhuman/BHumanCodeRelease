/**
 * @file Measurements.h
 *
 * Collection of functions that are used when dealing with measurements
 *
 * @author Tim Laue
 */

#pragma once

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Covariance.h"

namespace Measurements
{
  inline const Matrix2f positionToCovarianceMatrixInRobotCoordinates(
                                            const Vector2f& pointRelativeToRobot, float pointZInWorld,
                                            const Pose3f& cameraMatrix, const Pose3f& inverseCameraMatrix,
                                            const Vector2f& rotationDeviation)
  {
    const Vector3f unscaledVectorToPoint = inverseCameraMatrix * Vector3f(pointRelativeToRobot.x(), pointRelativeToRobot.y(), pointZInWorld);
    const Vector3f unscaledWorld = cameraMatrix.rotation * unscaledVectorToPoint;
    const float h = cameraMatrix.translation.z() - pointZInWorld;
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
    Covariance::fixCovariance(cov);
    return cov;
  }
};

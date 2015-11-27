/**
 * @file HeadLimits.cpp
 * Implementation of the methods of a struct for representing the limits of the head joints.
 * @author Felix Wenk
 */

#include "HeadLimits.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Eigen.h"

#include <algorithm>

Vector2f HeadLimits::getTiltBound(float pan) const
{
  const std::vector<float>::const_iterator begin = intervals.begin();
  const std::vector<float>::const_iterator end = intervals.end();
  const std::vector<float>::const_iterator it = std::lower_bound(begin, end, pan);

  if(it == end)
    return Vector2f(JointAngles::off, JointAngles::off); // Unreachable pan angle

  const size_t index = it - begin;
  const float xe = intervals[index];   // Interval end
  const float le = lowerBounds[index]; // Lower bound at interval end
  const float ue = upperBounds[index]; // Upper bound at interval end
  if(pan == xe)
    return Vector2f(ue, le);

  if(index == 0)
    return Vector2f(JointAngles::off, JointAngles::off); // Unreachable pan angle (smaller than begin of first interval)

  const float xs = intervals[index - 1];   // Interval start
  const float ls = lowerBounds[index - 1]; // Lower bound at interval start.
  const float us = upperBounds[index - 1]; // Upper bound at interval start

  const float lowerSlope = (le - ls) / (xe - xs);
  const float upperSlope = (ue - us) / (xe - xs);
  return Vector2f(us + upperSlope * (pan - xs),
                  ls + lowerSlope * (pan - xs));
}

bool HeadLimits::imageCenterHiddenByShoulder(const RobotCameraMatrix& robotCameraMatrix,
    const Vector3f& shoulderInOrigin,
    const float imageTilt, const float hysteresis) const
{
  Vector3f intersection = Vector3f::Zero();
  if(!intersectionWithShoulderPlane(robotCameraMatrix, shoulderInOrigin, imageTilt, intersection))
    return false; // No intersection with shoulder plane and therefore no intersection with circle.
  return intersection.norm() <= shoulderRadius + hysteresis;
}

bool HeadLimits::intersectionWithShoulderEdge(const RobotCameraMatrix& robotCameraMatrix,
    const Vector3f& shoulderInOrigin,
    Vector3f& intersection) const
{
  if(!intersectionWithShoulderPlane(robotCameraMatrix, shoulderInOrigin, 0.0f, intersection))
    return false; // No intersection with the plane.
  if(std::abs(intersection.x()) > shoulderRadius)
    return false; // No intersection with the shoulder circle.
  intersection.z() = shoulderRadius * std::sin(std::acos(intersection.x() / shoulderRadius));
  intersection += shoulderInOrigin;
  return true;
}

bool HeadLimits::intersectionWithShoulderPlane(const RobotCameraMatrix& robotCameraMatrix,
    const Vector3f& shoulderInOrigin,
    const float imageTilt, Vector3f& intersection) const
{
  static const Vector3f normal(0.0, 1.0f, 0.0f);
  Pose3f camera2Shoulder(-shoulderInOrigin);
  camera2Shoulder.conc(robotCameraMatrix);
  Vector3f line(std::cos(-imageTilt), 0.0f, std::sin(-imageTilt));
  line = camera2Shoulder * line - camera2Shoulder.translation;
  const float denominator = normal.dot(line);
  if(denominator == 0.0f)
    return false; // Line is parallel to the shoulder plane
  const float scale = (normal.dot(camera2Shoulder.translation)) / denominator;
  intersection = camera2Shoulder.translation - line * scale;
  return true;
}

void HeadLimits::draw() const
{
  DECLARE_DEBUG_DRAWING3D("representation:HeadLimits:left", "lShoulderPitch");
  DECLARE_DEBUG_DRAWING3D("representation:HeadLimits:right", "rShoulderPitch");
  SPHERE3D("representation:HeadLimits:left", 0, 0, 0, shoulderRadius, ColorRGBA::orange);
  SPHERE3D("representation:HeadLimits:right", 0, 0, 0, shoulderRadius, ColorRGBA::orange);
}

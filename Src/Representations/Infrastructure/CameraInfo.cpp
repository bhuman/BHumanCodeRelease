/**
 * @file CameraInfo.cpp
 * Implementation of struct CameraInfo
 */

#include "CameraInfo.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/RotationMatrix.h"
#include "Tools/Debugging/DebugDrawings3D.h"

void CameraInfo::updateFocalLength()
{
  focalLength = width / (2.f * std::tan(openingAngleWidth / 2.f));
  focalLengthInv = 1.f / focalLength;
  focalLenPow2 = sqr(focalLength);

  focalLengthHeight = height / (2.f * std::tan(openingAngleHeight / 2.f));
  focalLengthHeightInv = 1.f / focalLengthHeight;
}

void CameraInfo::draw() const
{
  DECLARE_DEBUG_DRAWING3D("FieldOfView", "camera");
  const float length = 5000.f;
  const RotationMatrix y(AngleAxisf(openingAngleHeight / 2.f, Vector3f(0, 1.f, 0)));
  const RotationMatrix x(AngleAxisf(openingAngleHeight / 2.f, Vector3f(0, 0, 1.f)));
  const RotationMatrix neg_y(AngleAxisf(-openingAngleHeight / 2.f, Vector3f(0, 1.f, 0)));
  const RotationMatrix neg_x(AngleAxisf(-openingAngleHeight / 2.f, Vector3f(0, 0, 1.f)));
  const Vector3f p0 = Vector3f::Zero();
  const Vector3f p1 = y * x * Vector3f(length, 0, 0);
  const Vector3f p2 = neg_y * x * Vector3f(length, 0, 0);
  const Vector3f p3 = y * neg_x * Vector3f(length, 0, 0);
  const Vector3f p4 = neg_y * neg_x * Vector3f(length, 0, 0);
  ColorRGBA color = camera == upper ? ColorRGBA::blue : ColorRGBA::red;
  color.a = 180;
  QUAD3D("FieldOfView", p0, p0, p1, p2, color);
  QUAD3D("FieldOfView", p0, p0, p2, p4, color);
  QUAD3D("FieldOfView", p0, p0, p3, p4, color);
  QUAD3D("FieldOfView", p0, p0, p3, p1, color);

  //Drawings just rendering one side, so we duplicate all
  QUAD3D("FieldOfView", p0, p0, p2, p1, color);
  QUAD3D("FieldOfView", p0, p0, p4, p2, color);
  QUAD3D("FieldOfView", p0, p0, p4, p3, color);
  QUAD3D("FieldOfView", p0, p0, p1, p3, color);
}

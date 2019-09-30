/**
 * @file CNSTools.cpp
 *
 * This file implements tools to translate between B-Human data types and
 * data types used in the contour detector implementation.
 *
 * @author Thomas Röfer
 */

#include "CNSTools.h"

namespace CNS
{
  Pose3f toPose3f(const Matrix4d& m)
  {
    RotationMatrix r;
    r << static_cast<float>(m(0, 0)), static_cast<float>(m(0, 1)), static_cast<float>(m(0, 2)),
         static_cast<float>(m(1, 0)), static_cast<float>(m(1, 1)), static_cast<float>(m(1, 2)),
         static_cast<float>(m(2, 0)), static_cast<float>(m(2, 1)), static_cast<float>(m(2, 2));
    return Pose3f(r, Vector3f(static_cast<float>(m(0, 3)), static_cast<float>(m(1, 3)), static_cast<float>(m(2, 3))));
  }

  Matrix4d toMatrix4d(const Pose3f& p)
  {
    const Vector3f& t = p.translation;
    const Matrix3f& r = p.rotation;
    Matrix4d result;
    result << r(0, 0), r(0, 1), r(0, 2), t.x(),
              r(1, 0), r(1, 1), r(1, 2), t.y(),
              r(2, 0), r(2, 1), r(2, 2), t.z(),
              0,       0,       0,       1;
    return result;
  }

  CameraModelOpenCV toCameraModelOpenCV(const CameraInfo& cameraInfo, const CameraIntrinsics& cameraIntrinsics)
  {
    return CameraModelOpenCV(Eigen::Isometry3d::Identity(),
                             cameraInfo.width,
                             cameraInfo.height,
                             cameraInfo.width / 2.f / std::tan(cameraIntrinsics.cameras[cameraInfo.camera].openingAngleWidth / 2.f),
                             cameraInfo.height / 2.f / std::tan(cameraIntrinsics.cameras[cameraInfo.camera].openingAngleHeight / 2.f),
                             cameraInfo.width * cameraIntrinsics.cameras[cameraInfo.camera].opticalCenter.x(),
                             cameraInfo.height * cameraIntrinsics.cameras[cameraInfo.camera].opticalCenter.y());
  }
}

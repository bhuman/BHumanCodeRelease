/**
 * @file OpenGLTools.h
 * Utility functions for using OpenGL
 * @author Colin Graf
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Math/RotationMatrix.h"

class OpenGLTools
{
public:
  /** Converts a pose to the OpenGL format
   * @param pose The pose to convert
   * @param transformation The converted pose
   */
  static void convertTransformation(const Pose3f& pose, float transformation[16])
  {
    transformation[0] = pose.rotation(0, 0);
    transformation[1] = pose.rotation(1, 0);
    transformation[2] = pose.rotation(2, 0);
    transformation[3] = 0.f;
    transformation[4] = pose.rotation(0, 1);
    transformation[5] = pose.rotation(1, 1);
    transformation[6] = pose.rotation(2, 1);
    transformation[7] = 0.f;
    transformation[8] = pose.rotation(0, 2);
    transformation[9] = pose.rotation(1, 2);
    transformation[10] = pose.rotation(2, 2);
    transformation[11] = 0.f;
    transformation[12] = pose.translation.x();
    transformation[13] = pose.translation.y();
    transformation[14] = pose.translation.z();
    transformation[15] = 1.f;
  }

  /** Converts a pose to the OpenGL format
   * @param rotation The rotational part of the pose to convert
   * @param translation The translational part of the pose to convert
   * @param transformation The converted pose
   */
  static void convertTransformation(const RotationMatrix* rotation, const Vector3f* translation, float transformation[16])
  {
    if(rotation)
    {
      transformation[0] = (*rotation)(0, 0);
      transformation[1] = (*rotation)(1, 0);
      transformation[2] = (*rotation)(2, 0);
      transformation[4] = (*rotation)(0, 1);
      transformation[5] = (*rotation)(1, 1);
      transformation[6] = (*rotation)(2, 1);
      transformation[8] = (*rotation)(0, 2);
      transformation[9] = (*rotation)(1, 2);
      transformation[10] = (*rotation)(2, 2);
    }
    else
    {
      transformation[0] = transformation[5] = transformation[10] = 1.f;
      transformation[1] = transformation[2] = transformation[4] = transformation[6] = transformation[8] = transformation[9] = 0.f;
    }
    if(translation)
    {
      transformation[12] = translation->x();
      transformation[13] = translation->y();
      transformation[14] = translation->z();
    }
    else
      transformation[12] = transformation[13] = transformation[14] = 0.f;
    transformation[3] = transformation[7] = transformation[11] = 0.f;
    transformation[15] = 1.f;
  }

  /**
   * Computes a camera transformation (basically like gluLookAt)
   */
  static void computeCameraTransformation(const Vector3f& eyePosition3D, const Vector3f& center3D, const Vector3f& upVector3D, float* transformation)
  {
     Vector3f forward, side, up;
     forward = center3D - eyePosition3D;
     forward.normalize();
     side = forward.cross(upVector3D);
     side.normalize();
     up = side.cross(forward);

     transformation[0] = side.x();
     transformation[4] = side.y();
     transformation[8] = side.z();
     transformation[1] = up.x();
     transformation[5] = up.y();
     transformation[9] = up.z();
     transformation[2] = -forward.x();
     transformation[6] = -forward.y();
     transformation[10] = -forward.z();
     transformation[12] = -transformation[0]*eyePosition3D.x() - transformation[4]*eyePosition3D.y() - transformation[8]*eyePosition3D.z();
     transformation[13] = -transformation[1]*eyePosition3D.x() - transformation[5]*eyePosition3D.y() - transformation[9]*eyePosition3D.z();
     transformation[14] = -transformation[2]*eyePosition3D.x() - transformation[6]*eyePosition3D.y() - transformation[10]*eyePosition3D.z();
     transformation[3] = transformation[7] = transformation[11] = 0.f;
     transformation[15] = 1.f;
  }

  /**
   * Computes a perspective projection matrix (basically like gluPerspective)
   * @param fovy angle of view in y-direction (in radian)
   * @param aspect An aspect ratio to determine the angle of view in x-direction
   * @param near The distance from the viewer to the near clipping plane
   * @param far The distance from the viewer to the far clipping plane
   * @param matrix The resulting matrix
   */
  static void computePerspective(float fovy, float aspect, float near, float far, float matrix[]);
};

/**
* @file OpenGLTools.h
* Utility functions for using OpenGL
* @author Colin Graf
*/

#pragma once

#include "Pose3.h"

class OpenGLTools
{
public:
  /** Converts a pose to the OpenGL format
  * @param pose The pose to convert
  * @param transformation The converted pose
  */
  static void convertTransformation(const Pose3<>& pose, float transformation[16])
  {
    transformation[0] = (float)pose.rotation.c0.x;
    transformation[1] = (float)pose.rotation.c0.y;
    transformation[2] = (float)pose.rotation.c0.z;
    transformation[3] = (float)0;
    transformation[4] = (float)pose.rotation.c1.x;
    transformation[5] = (float)pose.rotation.c1.y;
    transformation[6] = (float)pose.rotation.c1.z;
    transformation[7] = (float)0;
    transformation[8] = (float)pose.rotation.c2.x;
    transformation[9] = (float)pose.rotation.c2.y;
    transformation[10] = (float)pose.rotation.c2.z;
    transformation[11] = (float)0;
    transformation[12] = (float)pose.translation.x;
    transformation[13] = (float)pose.translation.y;
    transformation[14] = (float)pose.translation.z;
    transformation[15] = (float)1;
  }

  /** Converts a pose to the OpenGL format
  * @param rotation The rotational part of the pose to convert
  * @param translation The translational part of the pose to convert
  * @param transformation The converted pose
  */
  static void convertTransformation(const Matrix3x3<>* rotation, const Vector3<>* translation, float transformation[16])
  {
    if(rotation)
    {
      transformation[0] = (float)rotation->c0.x;
      transformation[1] = (float)rotation->c0.y;
      transformation[2] = (float)rotation->c0.z;
      transformation[4] = (float)rotation->c1.x;
      transformation[5] = (float)rotation->c1.y;
      transformation[6] = (float)rotation->c1.z;
      transformation[8] = (float)rotation->c2.x;
      transformation[9] = (float)rotation->c2.y;
      transformation[10] = (float)rotation->c2.z;
    }
    else
    {
      transformation[0]=transformation[5]=transformation[10]= (float)1;
      transformation[1]=transformation[2]=transformation[4]=transformation[6]=transformation[8]=transformation[9]= (float)0;
    }
    if(translation)
    {
      transformation[12] = (float)translation->x;
      transformation[13] = (float)translation->y;
      transformation[14] = (float)translation->z;
    }
    else
      transformation[12]=transformation[13]=transformation[14]=(float)0;
    transformation[3]=transformation[7]=transformation[11]=(float)0;
    transformation[15]=(float)1;
  }

  /**
  * Computes a camera transformation (basically like gluLookAt)
  */
  static void computeCameraTransformation(const Vector3<>& eyePosition3D, const Vector3<>& center3D, const Vector3<>& upVector3D, float* transformation)
  {
     Vector3<> forward, side, up;
     forward = center3D - eyePosition3D;
     forward.normalize();
     side = forward ^ upVector3D;
     side.normalize();
     up = side ^ forward;

     transformation[0] = (float)side.x;
     transformation[4] = (float)side.y;
     transformation[8] = (float)side.z;
     transformation[1] = (float)up.x;
     transformation[5] = (float)up.y;
     transformation[9] = (float)up.z;
     transformation[2] = (float)-forward.x;
     transformation[6] = (float)-forward.y;
     transformation[10] = (float)-forward.z;
     transformation[12] = (float)(transformation[0]*-eyePosition3D.x-transformation[4]*eyePosition3D.y-transformation[8]*eyePosition3D.z);
     transformation[13] = (float)(transformation[1]*-eyePosition3D.x-transformation[5]*eyePosition3D.y-transformation[9]*eyePosition3D.z);
     transformation[14] = (float)(transformation[2]*-eyePosition3D.x-transformation[6]*eyePosition3D.y-transformation[10]*eyePosition3D.z);
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

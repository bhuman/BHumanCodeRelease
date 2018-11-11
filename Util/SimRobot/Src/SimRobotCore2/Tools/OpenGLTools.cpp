/**
* @file OpenGLTools.cpp
* Utility functions for using OpenGL
* @author Colin Graf
*/

#include <cmath>

#include "OpenGLTools.h"

void OpenGLTools::computePerspective(float fovy, float aspect, float near, float far, float matrix[])
{
  matrix[5] = 1.f / std::tan(fovy * 0.5f);
  matrix[0] = matrix[5] / aspect;
  const float nearMFarInv = 1.f / (near - far);
  matrix[10] = (far + near) * nearMFarInv;
  matrix[11] = -1.f;
  matrix[14] = 2.f * far * near * nearMFarInv;
  matrix[1] = matrix[2] = matrix[3] = matrix[4] = matrix[6] = matrix[7] = matrix[8] = matrix[9] = matrix[12] = matrix[13] = matrix[15] = 0.f;
}

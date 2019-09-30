/**
 * @file RobotCameraMatrixProvider.cpp
 * This file implements a class to calculate the position of the camera relative to the body for the Nao.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * @author Colin Graf
 */

#include "RobotCameraMatrixProvider.h"

MAKE_MODULE(RobotCameraMatrixProvider, perception);

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix)
{
  robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions, theJointAngles.angles[Joints::headYaw], theJointAngles.angles[Joints::headPitch], theCameraCalibration, theCameraInfo.camera);
}

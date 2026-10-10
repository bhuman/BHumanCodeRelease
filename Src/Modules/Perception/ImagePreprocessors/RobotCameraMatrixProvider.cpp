/**
 * @file RobotCameraMatrixProvider.cpp
 * This file implements a class to calculate the position of the camera relative to the body.
 * @author Alexander Härtl
 * @author Colin Graf
 */

#include "RobotCameraMatrixProvider.h"

MAKE_MODULE(RobotCameraMatrixProvider);

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix)
{
  robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions, theJointAngles.angles[Joints::headYaw], theJointAngles.angles[Joints::headPitch], theCameraCalibration);
}

/**
 * @file RobotCameraMatrixProvider.cpp
 * This file implements a class to calculate the position of the camera relative to the body for the Nao.
 * @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
 * @author Colin Graf
 */

#include "RobotCameraMatrixProvider.h"

MAKE_MODULE(RobotCameraMatrixProvider);

void RobotCameraMatrixProvider::update(RobotCameraMatrix& robotCameraMatrix)
{
  robotCameraMatrix.computeRobotCameraMatrix(theRobotDimensions, theRobotModel.limbs[Limbs::head], theCameraCalibration, theCameraInfo.camera);
}

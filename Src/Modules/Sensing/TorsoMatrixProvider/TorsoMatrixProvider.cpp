/**
 * @file TorsoMatrixProvider.cpp
 * Implementation of module TorsoMatrixProvider.
 * @author Colin Graf
 */

#include "TorsoMatrixProvider.h"

MAKE_MODULE(TorsoMatrixProvider, sensing);

void TorsoMatrixProvider::update(TorsoMatrix& torsoMatrix)
{
  torsoMatrix.setTorsoMatrix(theInertialData, theRobotModel, theGroundContactState);
}

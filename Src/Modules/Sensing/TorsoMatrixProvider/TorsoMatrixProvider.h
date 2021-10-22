/**
 * @file TorsoMatrixProvider.h
 * Declaration of module TorsoMatrixProvider.
 * @author Colin Graf
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(TorsoMatrixProvider,
{,
  REQUIRES(GroundContactState),
  REQUIRES(InertialData),
  REQUIRES(RobotModel),
  PROVIDES(TorsoMatrix),
});

/**
 * @class TorsoMatrixProvider
 * A module that provides the (estimated) position and velocity of the inertia board.
 */
class TorsoMatrixProvider : public TorsoMatrixProviderBase
{
private:
  /**
   * Updates the TorsoMatrix representation.
   * @param torsoMatrix The inertia matrix representation which is updated by this module.
   */
  void update(TorsoMatrix& torsoMatrix) override;
};

/**
* @file TorsoMatrixProvider.h
* Declaration of module TorsoMatrixProvider.
* @author Colin Graf
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/OrientationData.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/MotionControl/OdometryData.h"

MODULE(TorsoMatrixProvider)
  REQUIRES(OrientationData)
  REQUIRES(RobotModel)
  REQUIRES(RobotDimensions)
  REQUIRES(GroundContactState)
  PROVIDES_WITH_MODIFY(TorsoMatrix)
  USES(TorsoMatrix)
  PROVIDES_WITH_MODIFY_AND_OUTPUT(OdometryData)
END_MODULE

/**
* @class TorsoMatrixProvider
* A module that provides the (estimated) position and velocity of the inertia board.
*/
class TorsoMatrixProvider : public TorsoMatrixProviderBase
{
private:
  float lastLeftFootZRotation; /**< The last z-rotation of the left foot. */
  float lastRightFootZRotation; /**< The last z-rotation of the right foot. */

  Vector3<> lastFootSpan; /**< The last span between both feet. */
  Pose3D lastTorsoMatrix; /**< The last torso matrix for calculating the odometry offset. */

  /** Updates the TorsoMatrix representation.
  * @param torsoMatrix The inertia matrix representation which is updated by this module.
  */
  void update(TorsoMatrix& torsoMatrix);

  /** Updates the OdometryData representation.
  * @param odometryData The odometry data representation which is updated by this module.
  */
  void update(OdometryData& odometryData);
};

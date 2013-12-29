#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Sensing/FsrZmp.h"

#include "Tools/RingBufferWithSum.h"

MODULE(FsrZmpProvider)
REQUIRES(RobotModel)
REQUIRES(SensorData)
REQUIRES(RobotDimensions)
PROVIDES_WITH_MODIFY(FsrZmp)
END_MODULE

/**
 * @class FsrZmpProvider
 * Computes the center of pressure for the left foot, right foot and whole robot
 * using a simple weighted sum.
 */
class FsrZmpProvider : public FsrZmpProviderBase
{
public:
  /** Default constructor. */
  FsrZmpProvider();

private:
  Vector3<> fsrBasePositions[8];/**< Coordinates of the fsr sensors relative to the ankles. Lfs are relative to left ankle, Rfs to right ankle*/
  void update(FsrZmp& fsrData);
  
  /**Calculate the current fsr positions for the given limb and store it in positions starting at startIndex*/
  void calculateFsrPosition(MassCalibration::Limb limb, int startIndex, Vector3<> (&positions)[8]) const;
};

/**
 * @file SoleHeightProvider.h
 *
 * @author Philip Reichenberg
 */

#pragma once

#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Sensing/SoleHeightDifference.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(SoleHeightProvider,
{,
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(SoleHeightDifference),
});

class SoleHeightProvider : public SoleHeightProviderBase
{
  void update(SoleHeightDifference& theSoleHeightDifference) override;
};

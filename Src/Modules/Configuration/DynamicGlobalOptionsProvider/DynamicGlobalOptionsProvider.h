/**
 * @file DynamicGlobalOptionsProvider.h
 *
 * This file does something very hacky
 *
 * @author Beeernd
 *
 */

#pragma once

#include <vector>
#include "Tools/Module/Module.h"
#include "Representations/Configuration/GlobalOptions.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Math/Eigen.h"

MODULE(DynamicGlobalOptionsProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  PROVIDES(GlobalOptions),

  LOADS_PARAMETERS(
  {,
    (bool) mirror,
    (std::vector<Vector4f>) areas,
    (float) bufferSize,
  }),
});

class DynamicGlobalOptionsProvider : public DynamicGlobalOptionsProviderBase
{
private:
  void update(GlobalOptions& options) override;
};

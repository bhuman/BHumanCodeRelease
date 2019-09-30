/**
 * @file GlobalFieldCoverageProvider.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/HulkFieldCoverage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/Module/Module.h"

MODULE(GlobalFieldCoverageProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(CognitionStateChanges),
  REQUIRES(FieldCoverage),
  REQUIRES(HulkFieldCoverage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),

  PROVIDES(GlobalFieldCoverage),
});

class GlobalFieldCoverageProvider : public GlobalFieldCoverageProviderBase
{
  void update(GlobalFieldCoverage& globalFieldCoverage) override;

  bool initDone = false;
  void init(GlobalFieldCoverage& globalFieldCoverage);

  void setCoverageAtFieldPosition(GlobalFieldCoverage& globalFieldCoverage, const Vector2f& positionOnField, const int coverage) const;
};

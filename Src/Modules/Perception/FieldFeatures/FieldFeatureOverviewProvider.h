/**
 * @file FieldFeatureOverviewProvider.h
 * Provides FieldFeatureOverview.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Perception/FieldFeatures/MidCircle.h"
#include "Representations/Perception/FieldFeatures/MidCorner.h"
#include "Representations/Perception/FieldFeatures/OuterCorner.h"
#include "Representations/Perception/FieldFeatures/PenaltyArea.h"
#include "Representations/Perception/FieldFeatures/PenaltyMarkWithPenaltyAreaLine.h"

MODULE(FieldFeatureOverviewProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(MidCircle),
  REQUIRES(MidCorner),
  REQUIRES(OuterCorner),
  REQUIRES(PenaltyArea),
  REQUIRES(PenaltyMarkWithPenaltyAreaLine),
  PROVIDES(FieldFeatureOverview),
});

class FieldFeatureOverviewProvider : public FieldFeatureOverviewProviderBase
{
  void update(FieldFeatureOverview& fieldFeatureOverview) override;
};

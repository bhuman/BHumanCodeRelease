/**
* @file GlobalFieldCoverageProvider.h
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Representations/Communication/TeammateData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Tools/Module/Module.h"

MODULE(GlobalFieldCoverageProvider,
{,
  REQUIRES(FieldCoverage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(TeammateData),
  PROVIDES(GlobalFieldCoverage),
});

class GlobalFieldCoverageProvider : public GlobalFieldCoverageProviderBase
{
public:
  GlobalFieldCoverageProvider();

  void update(GlobalFieldCoverage& globalFieldCoverage);

private:
  bool initDone = false;
  void init(GlobalFieldCoverage& globalFieldCoverage);
};
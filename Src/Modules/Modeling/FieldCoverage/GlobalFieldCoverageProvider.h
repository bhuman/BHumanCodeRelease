/**
 * @file GlobalFieldCoverageProvider.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/Module/Module.h"

MODULE(GlobalFieldCoverageProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(CognitionStateChanges),
  REQUIRES(FieldCoverage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(TeamData),

  PROVIDES(GlobalFieldCoverage),

  DEFINES_PARAMETERS(
  {,
    (int)(18) numOfCellsX,
    (int)(12) numOfCellsY,

    (int)(5000) maxTimeToBallDropIn,
    (float)(1000.f) dropInPenaltyDistance,
  }),
});

class GlobalFieldCoverageProvider : public GlobalFieldCoverageProviderBase
{
public:
  GlobalFieldCoverageProvider();

  void update(GlobalFieldCoverage& globalFieldCoverage);

private:
  float cellLengthX;
  float cellLengthY;

  unsigned lastTimeBallWentOut = 0;
  Vector2f ballOutPosition = Vector2f::Zero();

  Vector2f lastBallPosition = Vector2f::Zero();
  Vector2f lastBallPositionInsideField = Vector2f::Zero();
  Vector2f lastBallPositionLyingInsideField = Vector2f::Zero();

  bool initDone = false;
  void init(GlobalFieldCoverage& globalFieldCoverage);

  void accountForBallDropIn(GlobalFieldCoverage& globalFieldCoverage);
  void calculateDropInPosition(GlobalFieldCoverage& globalFieldCoverage);

  void setCoverageAtFieldPosition(GlobalFieldCoverage& globalFieldCoverage, const Vector2f& positionOnField, const int coverage) const;
};

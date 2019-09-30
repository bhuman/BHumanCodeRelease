/**
 * @file HulkFieldCoverageProvider.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/HulkFieldCoverage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Tools/Module/Module.h"

MODULE(HulkFieldCoverageProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(CameraInfo),
  REQUIRES(FieldCoverage),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(TeamData),

  PROVIDES(HulkFieldCoverage),

  DEFINES_PARAMETERS(
  {,
    (int)(18) numOfCellsX,
    (int)(12) numOfCellsY,

    (float)(2000.f) ballVisibilityMax,
    (float)(500.f)  ballVisibilityMin,
    (float)(1200.f) ballVisibilitylaterallyMin,
    (Angle)(50_deg) shoulderRange,
    (float)(200.f)  obstacleRadius,
  }),
});

class HulkFieldCoverageProvider : public HulkFieldCoverageProviderBase
{
public:
  HulkFieldCoverageProvider();

  void update(HulkFieldCoverage& hulkFieldCoverage) override;

private:
  struct Cell
  {
    unsigned& timestamp;
    Vector2f positionOnField;
    Vector2f polygon[4];

    Cell(unsigned& timestamp, float positionOnFieldX, float positionOnFieldY, float cellLengthX, float cellLengthY)
      : timestamp(timestamp), positionOnField(positionOnFieldX, positionOnFieldY)
    {
      const float xMin = positionOnFieldX - cellLengthX / 2.f;
      const float xMax = positionOnFieldX + cellLengthX / 2.f;
      const float yMin = positionOnFieldY - cellLengthY / 2.f;
      const float yMax = positionOnFieldY + cellLengthY / 2.f;

      polygon[0] = Vector2f(xMin + 25.0f, yMin + 25.0f);
      polygon[1] = Vector2f(xMin + 25.0f, yMax - 25.0f);
      polygon[2] = Vector2f(xMax - 25.0f, yMax - 25.0f);
      polygon[3] = Vector2f(xMax - 25.0f, yMin + 25.0f);
    }
  };
  std::vector<Cell> grid;
  float cellLengthX;
  float cellLengthY;

  bool initDone = false;
  void init(HulkFieldCoverage& hulkFieldCoverage);

  bool isViewBlocked(const Angle angle, const float sqrDistance) const;
  void calculateObstacles(const Teammate& teammate);

  struct CoverageObstacle
  {
    const Angle angleLeft;
    const Angle angleRight;
    const float sqrDistance;

    CoverageObstacle(const Angle angleLeft, const Angle angleRight, const float sqrDistance)
      : angleLeft(angleLeft), angleRight(angleRight), sqrDistance(sqrDistance) {}
  };
  std::vector<CoverageObstacle> obstacles;
  void draw() const;
};

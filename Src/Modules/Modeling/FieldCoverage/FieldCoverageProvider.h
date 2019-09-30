/**
 * @file FieldCoverageProvider.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"

#include <vector>

MODULE(FieldCoverageProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(BHumanMessageOutputGenerator),

  PROVIDES_WITHOUT_MODIFY(FieldCoverage),

  DEFINES_PARAMETERS(
  {,
    (int)(18) numOfCellsX,
    (int)(12) numOfCellsY,

    (float)(2000.f) ballVisibilityRange,
    (float)(600000.f)  squaredBallVisibilityMin,
    (Angle)(50_deg) shoulderRange,
    (float)(200.f) obstacleRadius,
  }),
});

class FieldCoverageProvider : public FieldCoverageProviderBase
{
public:
  FieldCoverageProvider();

  void update(FieldCoverage& fieldCoverage) override;

private:
  struct InternalCell
  {
    unsigned& timestamp;
    Vector2f positionOnField;
    Vector2f polygon[4];

    InternalCell(unsigned& timestamp, float positionOnFieldX, float positionOnFieldY, float cellLengthX, float cellLengthY)
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
  std::vector<InternalCell> cells;

  struct CoverageObstacle
  {
    const Angle angleLeft;
    const Angle angleRight;
    const float sqrDistance;

    CoverageObstacle(const Angle angleLeft, const Angle angleRight, const float sqrDistance)
      : angleLeft(angleLeft), angleRight(angleRight), sqrDistance(sqrDistance) {}
  };
  std::vector<CoverageObstacle> obstacles;
  void calculateObstacles();

  bool isViewBlocked(const Angle angle, const float sqrDistance) const;

  int nextLineToCalculate = 0;

  bool initDone = false;
  void init(FieldCoverage& fieldCoverage);

  void draw() const;
};

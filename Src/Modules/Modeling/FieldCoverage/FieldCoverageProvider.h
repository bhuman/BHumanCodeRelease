/**
* @file FieldCoverageProvider.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#pragma once

#include "Tools/Module/Module.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Tools/Math/Geometry.h"
#include <vector>


MODULE(FieldCoverageProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),

  PROVIDES_WITHOUT_MODIFY(FieldCoverage),

  DEFINES_PARAMETERS(
  {,
    (float) (2000.f) ballVisibilityRange,
    (float) (200.f) obstacleRadius,
    (int) (3000) maxTimeToBallDropIn,
    (float) (1000.f) dropInPenaltyDistance,
  }),
});

class FieldCoverageProvider : public FieldCoverageProviderBase
{
public:
  FieldCoverageProvider();

  void update(FieldCoverage& fieldCoverage);

private:
  struct Cell
  {
    unsigned& timeStamp;
    unsigned& lastSeen;
    Vector2f positionOnField;
    Vector2f polygon[4];

    Cell(unsigned& timeStamp, unsigned& lastSeen, const float positionOnFieldX, const float positionOnFieldY, const float cellLengthX, const float cellLengthY)
      : timeStamp(timeStamp), lastSeen(lastSeen), positionOnField(positionOnFieldX, positionOnFieldY)
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
  std::vector<Cell> cells;

  struct CoverageObstacle
  {
    const Angle angleLeft;
    const Angle angleRight;
    const float sqrDistance;

    CoverageObstacle(const Angle angleLeft, const Angle angleRight, const float sqrDistance)
      : angleLeft(angleLeft), angleRight(angleRight), sqrDistance(sqrDistance) {}
  };
  std::vector<CoverageObstacle> obstacles;

  float cellLengthX = 500;
  float cellLengthY = 500;

  int calculateThisFrame = 0;

  unsigned lastTimeBallWasSeenInsideField = 0;
  unsigned lastTimeBallWasSeenOutsideField = 0;

  Vector2f lastBallPositionInsideField = Vector2f::Zero();
  Vector2f lastBallPositionOutsideField = Vector2f::Zero();
  Vector2f lastBallPositionLyingInsideField = Vector2f::Zero();

  unsigned lastTimeBallWentOut = 0;
  Vector2f ballOutPosition = Vector2f::Zero();

  void accountForBallDropIn(FieldCoverage& fieldCoverage);
  void calculateBallOutPosition();
  void calculateDropInPosition(FieldCoverage& fieldCoverage);

  void calculateObstacles();
  bool isViewBlocked(const Angle angle, const float sqrDistance);

  void accountForBallPosition(FieldCoverage& fieldCoverage);

  bool initDone = false;
  void init(FieldCoverage& fieldCoverage);

  void draw() const;
};
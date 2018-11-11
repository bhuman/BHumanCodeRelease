/**
 * @file HulkFieldCoverageProvider.cpp
 * @author Enno Roehrig based on GlobalFieldCoverageProvider by Andreas Stolpmann
 */
//TODO
//handle shoulder

#include "HulkFieldCoverageProvider.h"
#include <string>

MAKE_MODULE(HulkFieldCoverageProvider, modeling);

HulkFieldCoverageProvider::HulkFieldCoverageProvider()
{}

void HulkFieldCoverageProvider::update(HulkFieldCoverage& hulkFieldCoverage)
{
  if(!initDone)
    init(hulkFieldCoverage);

  for(const auto& teammate : theTeamData.teammates)
    if(teammate.mateType != Teammate::TeamOrigin::BHumanRobot&& teammate.status == Teammate::Status::PLAYING && teammate.isUpright)
    {
      const Angle openingAngle_2 = theCameraInfo.openingAngleWidth * 0.9f / 2.f;
      // This is wrong because headYawAngle is not transmitted.
      const Angle angleLeft = openingAngle_2;
      const Angle angleRight = -openingAngle_2;

      calculateObstacles(teammate);

      const float sqrBallVisibilityMax = ballVisibilityMax * ballVisibilityMax;
      const float sqrBallVisibilityMin = ballVisibilityMin * ballVisibilityMin;
      for(size_t i = 0; i < grid.size(); ++i)
      {
        Cell& c = grid[i];
        const Vector2f positionRelative = teammate.theRobotPose.inversePose * c.positionOnField;
        const Angle angle = positionRelative.angle();

        if(angleLeft > angle && angle > angleRight)
        {
          const float sqrDistance = positionRelative.squaredNorm();
          if(sqrDistance < sqrBallVisibilityMax && sqrDistance > sqrBallVisibilityMin && !isViewBlocked(angle, sqrDistance))
          {
            c.timestamp = theFrameInfo.time;
          }
        }
      }
    }
}

void HulkFieldCoverageProvider::calculateObstacles(const Teammate& teammate)
{
  obstacles.clear();
  obstacles.reserve(teammate.theObstacleModel.obstacles.size());

  for(const Obstacle& omo : teammate.theObstacleModel.obstacles)
  {
    if(omo.type != Obstacle::Type::goalpost)
    {
      Vector2f left, right;
      left = right = omo.center.normalized(obstacleRadius);
      left.rotateLeft();
      right.rotateRight();
      left += omo.center;
      right += omo.center;

      obstacles.emplace_back(left.angle(), right.angle(), omo.center.squaredNorm());
    }
  }
}

bool HulkFieldCoverageProvider::isViewBlocked(const Angle angle, const float sqrDistance) const
{
  for(const CoverageObstacle& o : obstacles)
  {
    if(o.sqrDistance < sqrDistance && o.angleLeft > angle && angle > o.angleRight)
      return true;
  }
  return (angle.diffAbs(0) > shoulderRange && sqrDistance < ballVisibilitylaterallyMin * ballVisibilitylaterallyMin);
}

void HulkFieldCoverageProvider::init(HulkFieldCoverage& hulkFieldCoverage)
{
  initDone = true;

  cellLengthX = theFieldDimensions.xPosOpponentGroundline * 2 / numOfCellsX;
  cellLengthY = theFieldDimensions.yPosLeftSideline * 2 / numOfCellsY;

  float positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
  float positionOnFieldY = theFieldDimensions.yPosRightSideline + cellLengthY / 2.f;

  const unsigned time = std::max(10000u, theFrameInfo.time);

  hulkFieldCoverage.lines.resize(numOfCellsY);
  for(size_t y = 0; y < hulkFieldCoverage.lines.size(); ++y)
  {
    hulkFieldCoverage.lines[y].y = static_cast<int>(y);
    hulkFieldCoverage.lines[y].timestamps.resize(numOfCellsX, time);
  }

  grid.reserve(numOfCellsY * numOfCellsX);
  for(int y = 0; y < numOfCellsY; ++y)
  {
    for(int x = 0; x < numOfCellsX; ++x)
    {
      grid.emplace_back(hulkFieldCoverage.lines[y].timestamps[x], positionOnFieldX, positionOnFieldY, cellLengthX, cellLengthY);
      positionOnFieldX += cellLengthX;
    }
    positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
    positionOnFieldY += cellLengthY;
  }
}
void HulkFieldCoverageProvider::draw() const
{
  DECLARE_DEBUG_DRAWING("module:HulkFieldCoverageProvider:fieldCoverage", "drawingOnField");
  COMPLEX_DRAWING("module:HulkFieldCoverageProvider:fieldCoverage")
  {
    unsigned min = std::numeric_limits<unsigned>::max();
    unsigned max = std::numeric_limits<unsigned>::min();
    for(size_t i = 0; i < grid.size(); ++i)
    {
      if(min > grid[i].timestamp)
        min = grid[i].timestamp;
      if(max < grid[i].timestamp)
        max = grid[i].timestamp;
    }

    for(size_t i = 0; i < grid.size(); ++i)
    {
      const Cell& c = grid[i];
      const unsigned char alpha = static_cast<unsigned char>((1.f - (static_cast<float>(c.timestamp - min) / static_cast<float>(max - min))) * 255.f);
      const ColorRGBA color(255, 0, 0, alpha);
      QUADRANGLE("module:FieldCoverageProvider:fieldCoverage",
                 c.polygon[0].x(), c.polygon[0].y(),
                 c.polygon[1].x(), c.polygon[1].y(),
                 c.polygon[2].x(), c.polygon[2].y(),
                 c.polygon[3].x(), c.polygon[3].y(),
                 20, Drawings::solidPen, color);
    }
  }
}

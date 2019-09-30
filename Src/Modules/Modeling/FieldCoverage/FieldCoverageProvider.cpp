/**
 * @file FieldCoverageProvider.cpp
 * @author Andreas Stolpmann
 */

#include "FieldCoverageProvider.h"

#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(FieldCoverageProvider, modeling);

FieldCoverageProvider::FieldCoverageProvider()
{}

void FieldCoverageProvider::update(FieldCoverage& fieldCoverage)
{
  if(!initDone)
    init(fieldCoverage);

  if(theCameraMatrix.isValid && theRobotPose.deviation < 60.f)
  {
    const Angle openingAngle_2 = theCameraInfo.openingAngleWidth * 0.9f / 2.f;
    const Angle headAngle = std::atan2(theCameraMatrix.rotation(1, 0), theCameraMatrix.rotation(0, 0));
    const Angle angleLeft = headAngle + openingAngle_2;
    const Angle angleRight = headAngle - openingAngle_2;

    calculateObstacles();

    const float sqrBallVisibilityRange = ballVisibilityRange * ballVisibilityRange;
    for(int i = nextLineToCalculate * numOfCellsX; i < (nextLineToCalculate + 1) * numOfCellsX; ++i)
    {
      InternalCell& c = cells[i];
      const Vector2f positionRelative = theRobotPose.inversePose * c.positionOnField;
      const Angle angle = positionRelative.angle();

      if(angleLeft > angle && angle > angleRight)
      {
        const float sqrDistance = positionRelative.squaredNorm();
        if(sqrDistance < sqrBallVisibilityRange && !isViewBlocked(angle, sqrDistance))
        {
          c.timestamp = theFrameInfo.time;
        }
      }
    }
    nextLineToCalculate = (nextLineToCalculate + 1) % numOfCellsY;
  }

  draw();
}

void FieldCoverageProvider::calculateObstacles()
{
  obstacles.clear();
  obstacles.reserve(theObstacleModel.obstacles.size());

  for(const Obstacle& omo : theObstacleModel.obstacles)
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

bool FieldCoverageProvider::isViewBlocked(const Angle angle, const float sqrDistance) const
{
  for(const CoverageObstacle& o : obstacles)
  {
    if(o.sqrDistance < sqrDistance && o.angleLeft > angle && angle > o.angleRight)
      return true;
  }
  return (angle.diffAbs(0) > shoulderRange && sqrDistance < squaredBallVisibilityMin);
}

void FieldCoverageProvider::init(FieldCoverage& fieldCoverage)
{
  initDone = true;

  const float cellLengthX = theFieldDimensions.xPosOpponentGroundline * 2 / numOfCellsX;
  const float cellLengthY = theFieldDimensions.yPosLeftSideline * 2 / numOfCellsY;
  const unsigned time = std::max(10000u, theFrameInfo.time);

  for(size_t y = 0; y < fieldCoverage.lines.size(); ++y)
  {
    fieldCoverage.lines[y].y = static_cast<int>(y);
    fieldCoverage.lines[y].timestamps.resize(numOfCellsX, time);
  }

  float positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
  float positionOnFieldY = theFieldDimensions.yPosRightSideline + cellLengthY / 2.f;

  cells.reserve(numOfCellsY * numOfCellsX);
  for(int y = 0; y < numOfCellsY; ++y)
  {
    for(int x = 0; x < numOfCellsX; ++x)
    {
      cells.emplace_back(fieldCoverage.lines[y].timestamps[x], positionOnFieldX, positionOnFieldY, cellLengthX, cellLengthY);
      positionOnFieldX += cellLengthX;
    }
    positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
    positionOnFieldY += cellLengthY;
  }
}

void FieldCoverageProvider::draw() const
{
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:fieldCoverage", "drawingOnField");
  COMPLEX_DRAWING("module:FieldCoverageProvider:fieldCoverage")
  {
    unsigned min = std::numeric_limits<unsigned>::max();
    unsigned max = std::numeric_limits<unsigned>::min();
    for(size_t i = 0; i < cells.size(); ++i)
    {
      if(min > cells[i].timestamp)
        min = cells[i].timestamp;
      if(max < cells[i].timestamp)
        max = cells[i].timestamp;
    }

    for(size_t i = 0; i < cells.size(); ++i)
    {
      const InternalCell& c = cells[i];
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

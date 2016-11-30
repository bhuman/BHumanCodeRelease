/**
* @file FieldCoverageProvider.cpp
* @author <A href="mailto:andisto@tzi.de">Andreas Stolpmann</A>
*/

#include "FieldCoverageProvider.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Math/Transformation.h"


MAKE_MODULE(FieldCoverageProvider, modeling);

FieldCoverageProvider::FieldCoverageProvider()
{}

void FieldCoverageProvider::update(FieldCoverage& fieldCoverage)
{
  if(!initDone)
    init(fieldCoverage);

  if(++calculateThisFrame >= 5 && theCameraMatrix.isValid && theRobotPose.deviation < 60.f)
  {
    calculateThisFrame = 0;

    const Angle openingAngle_2 = theCameraInfo.openingAngleWidth * 0.9f / 2.f;
    const Angle headAngle = std::atan2(theCameraMatrix.rotation(1, 0), theCameraMatrix.rotation(0, 0));
    const Angle angleLeft = headAngle + openingAngle_2;
    const Angle angleRight = headAngle - openingAngle_2;

    calculateObstacles();

    const float sqrBallVisibilityRange = ballVisibilityRange * ballVisibilityRange;
    for(size_t i = 0; i < cells.size(); ++i)
    {
      Cell& c = cells[i];
      const Vector2f positionRelative = theRobotPose.inversePose * c.positionOnField;
      const Angle angle = positionRelative.angle();

      if(angleLeft > angle && angle > angleRight)
      {
        const float sqrDistance = positionRelative.squaredNorm();
        if(sqrDistance < sqrBallVisibilityRange && !isViewBlocked(angle, sqrDistance))
        {
          c.timeStamp = theFrameInfo.time;
          c.lastSeen = theFrameInfo.time;
        }
      }
    }
  }

  accountForBallPosition(fieldCoverage);
  accountForBallDropIn(fieldCoverage);

  draw();
}

void FieldCoverageProvider::accountForBallPosition(FieldCoverage& fieldCoverage)
{
  auto handleBallPosition = [&](const Vector2f& positionOnField)
  {
    if(theFieldDimensions.isInsideField(positionOnField))
    {
      ASSERT(std::isfinite(positionOnField.x()));
      ASSERT(std::isfinite(positionOnField.y()));
      const int x = static_cast<int>((positionOnField.x() - theFieldDimensions.xPosOwnGroundline) / cellLengthX);
      const int y = static_cast<int>((positionOnField.y() - theFieldDimensions.yPosRightSideline) / cellLengthY);
      const unsigned value = 200 * 300;
      fieldCoverage[y].timestamps[x] = theFrameInfo.time;
      fieldCoverage[y].values[x] = std::min(fieldCoverage[y].values[x], value > theFrameInfo.time ? 0 : theFrameInfo.time - value);
    }
  };

  if(theTeamBallModel.isValid)
    handleBallPosition(theTeamBallModel.position);
  if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 4000 && theBallModel.timeWhenDisappeared == theFrameInfo.time)
    handleBallPosition(theRobotPose * theBallModel.estimate.position);
}

void FieldCoverageProvider::accountForBallDropIn(FieldCoverage& fieldCoverage)
{
  if(theTeamBallModel.isValid)
  {
    if(theFieldDimensions.isInsideField(theTeamBallModel.position))
    {
      lastTimeBallWasSeenInsideField = theFrameInfo.time;
      lastBallPositionInsideField = theTeamBallModel.position;

      if(theTeamBallModel.velocity.squaredNorm() < 1.f)
        lastBallPositionLyingInsideField = theTeamBallModel.position;
    }
    else
    {
      lastTimeBallWasSeenOutsideField = theFrameInfo.time;
      lastBallPositionOutsideField = theTeamBallModel.position;
    }
  }

  if(theGameInfo.state == STATE_PLAYING && static_cast<int>(theGameInfo.dropInTime) * 1000 < maxTimeToBallDropIn)
  {
    calculateBallOutPosition();

    if(lastTimeBallWasSeenInsideField >= lastTimeBallWentOut && lastTimeBallWasSeenInsideField < lastTimeBallWasSeenOutsideField)
    {
      CROSS("module:FieldCoverageProvider:ballOutPosition",
            ballOutPosition.x(), ballOutPosition.y(), 75, 30,
            Drawings::solidPen, ColorRGBA(255, 192, 203));
    }

    calculateDropInPosition(fieldCoverage);

    const Vector2i dropInCell(static_cast<int>((fieldCoverage.ballDropInPosition.x() - theFieldDimensions.xPosOwnGroundline) / cellLengthX),
                              static_cast<int>((fieldCoverage.ballDropInPosition.y() - theFieldDimensions.yPosRightSideline) / cellLengthY));

    const int yOtherLine = FieldCoverage::numOfCellsY - 1 - dropInCell.y();

    const unsigned dropInLineTime = theFrameInfo.time - 30 * 300;
    const unsigned otherLineTime = theFrameInfo.time - FieldCoverage::numOfCellsX * 300;
    const unsigned baseTime = theFrameInfo.time - 5 * 300;

    for(int y = 0; y < FieldCoverage::numOfCellsY; ++y)
    {
      if(y == dropInCell.y())
      {
        for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
        {
          fieldCoverage[y].timestamps[x] = theFrameInfo.time;
          fieldCoverage[y].values[x] = dropInLineTime + (std::abs(dropInCell.x() - x) * 300);
        }
      }
      else if(y == yOtherLine)
      {
        for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
        {
          fieldCoverage[y].timestamps[x] = theFrameInfo.time;
          fieldCoverage[y].values[x] = otherLineTime;
        }
      }
      else
      {
        for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
        {
          fieldCoverage[y].timestamps[x] = theFrameInfo.time;
          fieldCoverage[y].values[x] = baseTime;
        }
      }
    }
  }
}

void FieldCoverageProvider::calculateDropInPosition(FieldCoverage& fieldCoverage)
{
  const bool ownTeamKicked = theGameInfo.dropInTeam == theOwnTeamInfo.teamNumber;

  fieldCoverage.ballDropInPosition.y() = lastBallPositionLyingInsideField.y() < 0 ? theFieldDimensions.yPosRightDropInLine : theFieldDimensions.yPosLeftDropInLine;
  fieldCoverage.ballDropInPosition.x() = lastBallPositionLyingInsideField.x() + (ownTeamKicked ? -dropInPenaltyDistance : dropInPenaltyDistance);

  if(lastTimeBallWasSeenInsideField >= lastTimeBallWentOut && lastTimeBallWasSeenInsideField < lastTimeBallWasSeenOutsideField && ballOutPosition != Vector2f::Zero())
  {
    fieldCoverage.ballDropInPosition.y() = ballOutPosition.y() < 0 ? theFieldDimensions.yPosRightDropInLine : theFieldDimensions.yPosLeftDropInLine;

    if(ballOutPosition.x() > theFieldDimensions.xPosOpponentGroundline - 10.0f && ballOutPosition.x() < theFieldDimensions.xPosOpponentGroundline + 10.0f)
      fieldCoverage.ballDropInPosition.x() = ownTeamKicked ? std::min(fieldCoverage.ballDropInPosition.x(), 0.f) : theFieldDimensions.xPosOpponentDropInLine;
    else if(ballOutPosition.x() > theFieldDimensions.xPosOwnGroundline - 10.0f && ballOutPosition.x() < theFieldDimensions.xPosOwnGroundline + 10.0f)
      fieldCoverage.ballDropInPosition.x() = ownTeamKicked ? theFieldDimensions.xPosOpponentDropInLine : std::max(fieldCoverage.ballDropInPosition.x(), 0.f);
    else
      fieldCoverage.ballDropInPosition.x() = ownTeamKicked ? std::min(fieldCoverage.ballDropInPosition.x(), fieldCoverage.ballDropInPosition.x() - dropInPenaltyDistance)
                                                           : std::max(fieldCoverage.ballDropInPosition.x(), fieldCoverage.ballDropInPosition.x() + dropInPenaltyDistance);
  }

  CROSS("module:FieldCoverageProvider:dropInPosition",
        fieldCoverage.ballDropInPosition.x(), fieldCoverage.ballDropInPosition.y(), 75, 30,
        Drawings::solidPen, ColorRGBA(255, 192, 203));
}

void FieldCoverageProvider::calculateBallOutPosition()
{
  if(lastTimeBallWasSeenInsideField <= lastTimeBallWentOut)
    return;

  if(lastTimeBallWasSeenInsideField > lastTimeBallWasSeenOutsideField)
    return;

  const Geometry::Line ballOutLine(lastBallPositionInsideField, lastBallPositionOutsideField - lastBallPositionInsideField);

  ballOutPosition = Vector2f::Zero();

  for(const auto& borderLine : theFieldDimensions.fieldBorder.lines)
  {
    if(Geometry::checkIntersectionOfLines(lastBallPositionInsideField, lastBallPositionOutsideField, borderLine.from, borderLine.to)
       && Geometry::getIntersectionOfLines(Geometry::Line(borderLine.from, borderLine.to - borderLine.from), ballOutLine, ballOutPosition))
    {
      lastTimeBallWentOut = lastTimeBallWasSeenInsideField;
      return;
    }
  }

  ASSERT(false);
}

void FieldCoverageProvider::calculateObstacles()
{
  obstacles.clear();
  obstacles.reserve(theObstacleModel.obstacles.size());

  for(const Obstacle& omo : theObstacleModel.obstacles)
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

bool FieldCoverageProvider::isViewBlocked(const Angle angle, const float sqrDistance)
{
  for(const CoverageObstacle& o : obstacles)
  {
    if(o.sqrDistance < sqrDistance && o.angleLeft > angle && angle > o.angleRight)
      return true;
  }
  return false;
}

void FieldCoverageProvider::init(FieldCoverage& fieldCoverage)
{
  initDone = true;

  for(int y = 0; y < FieldCoverage::numOfCellsY; ++y)
    fieldCoverage[y].y = y;

  cells.reserve(FieldCoverage::numOfCellsY * FieldCoverage::numOfCellsX);

  cellLengthX = theFieldDimensions.xPosOpponentGroundline / FieldCoverage::numOfCellsX * 2;
  cellLengthY = theFieldDimensions.yPosLeftSideline / FieldCoverage::numOfCellsY * 2;

  float positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
  float positionOnFieldY = theFieldDimensions.yPosRightSideline + cellLengthY / 2.f;

  const unsigned time = std::max(10000u, theFrameInfo.time);
  for(int y = 0; y < FieldCoverage::numOfCellsY; ++y)
  {
    for(int x = 0; x < FieldCoverage::numOfCellsX; ++x)
    {
      cells.emplace_back(fieldCoverage[y].timestamps[x], fieldCoverage[y].values[x], positionOnFieldX, positionOnFieldY, cellLengthX, cellLengthY);
      cells.back().timeStamp = time;
      cells.back().lastSeen = time;
      positionOnFieldX += cellLengthX;
    }
    positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
    positionOnFieldY += cellLengthY;
  }
}

void FieldCoverageProvider::draw() const
{
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:dropInPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:ballOutPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:FieldCoverageProvider:fieldCoverage", "drawingOnField");
  COMPLEX_DRAWING("module:FieldCoverageProvider:fieldCoverage")
  {
    auto alpha = [&](const int lastSeen)
    {
      const int alpha = (theFrameInfo.time - lastSeen) / 300;
      return static_cast<unsigned char>(std::min(alpha, 255));
    };

    for(size_t i = 0; i < cells.size(); ++i)
    {
      const Cell& c = cells[i];
      const ColorRGBA color(255, 0, 0, alpha(c.lastSeen));
      QUADRANGLE("module:FieldCoverageProvider:fieldCoverage",
                 c.polygon[0].x(), c.polygon[0].y(),
                 c.polygon[1].x(), c.polygon[1].y(),
                 c.polygon[2].x(), c.polygon[2].y(),
                 c.polygon[3].x(), c.polygon[3].y(),
                 20, Drawings::solidPen, color);
    }
  }
}

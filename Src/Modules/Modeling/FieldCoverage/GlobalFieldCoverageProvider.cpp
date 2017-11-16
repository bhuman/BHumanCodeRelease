/**
 * @file GlobalFieldCoverageProvider.cpp
 * @author Andreas Stolpmann
 */

#include "GlobalFieldCoverageProvider.h"
#include <string>

MAKE_MODULE(GlobalFieldCoverageProvider, modeling);

GlobalFieldCoverageProvider::GlobalFieldCoverageProvider()
{}

void GlobalFieldCoverageProvider::update(GlobalFieldCoverage& globalFieldCoverage)
{
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:dropInPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:GlobalFieldCoverageProvider:ballOutPosition", "drawingOnField");

  if(!initDone)
    init(globalFieldCoverage);

  if(theCognitionStateChanges.lastGameState != STATE_SET && theGameInfo.state == STATE_SET)
  {
    const int min = -static_cast<int>(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline).squaredNorm()) / 1000;
    for(size_t i = 0; i < globalFieldCoverage.grid.size(); ++i)
    {
      globalFieldCoverage.grid[i].timestamp = theFrameInfo.time;
      globalFieldCoverage.grid[i].coverage = min + static_cast<int>(globalFieldCoverage.grid[i].positionOnField.squaredNorm()) / 1000;
    }
  }

  auto addLine = [&](const FieldCoverage::GridLine& line)
  {
    const size_t base = line.y * line.timestamps.size();

    for(size_t x = 0; x < line.timestamps.size(); ++x)
    {
      if(line.timestamps[x] > globalFieldCoverage.grid[base + x].timestamp)
      {
        globalFieldCoverage.grid[base + x].timestamp = line.timestamps[x];
        globalFieldCoverage.grid[base + x].coverage = line.timestamps[x];
      }
    }
  };

  for(size_t y = 0; y < theFieldCoverage.lines.size(); ++y)
    addLine(theFieldCoverage.lines[y]);
  for(const auto teammate : theTeamData.teammates)
    if(!teammate.theFieldCoverage.lines.empty() && teammate.mateType == Teammate::TeamOrigin::BHumanRobot)
      addLine(teammate.theFieldCoverage.lines.back());

  if(theTeamBallModel.isValid)
    setCoverageAtFieldPosition(globalFieldCoverage, theTeamBallModel.position, 0);
  if(theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 4000 && theBallModel.timeWhenDisappeared == theFrameInfo.time)
    setCoverageAtFieldPosition(globalFieldCoverage, theRobotPose * theBallModel.estimate.position, 0);
  setCoverageAtFieldPosition(globalFieldCoverage, theRobotPose.translation, theFrameInfo.time);

  accountForBallDropIn(globalFieldCoverage);

  globalFieldCoverage.meanCoverage = 0;
  for(size_t i = 0; i < globalFieldCoverage.grid.size(); ++i)
    globalFieldCoverage.meanCoverage += globalFieldCoverage.grid[i].coverage;
  globalFieldCoverage.meanCoverage /= static_cast<int>(globalFieldCoverage.grid.size());
}

void GlobalFieldCoverageProvider::accountForBallDropIn(GlobalFieldCoverage& globalFieldCoverage)
{
  if(theTeamBallModel.isValid)
  {
    if(theFieldDimensions.isInsideField(theTeamBallModel.position))
    {
      lastBallPositionInsideField = theTeamBallModel.position;
      if(theTeamBallModel.velocity.squaredNorm() < 1.f)
        lastBallPositionLyingInsideField = theTeamBallModel.position;
    }
    else if(theFieldDimensions.isInsideField(lastBallPosition))
    {
      ballOutPosition = theTeamBallModel.position;
      lastTimeBallWentOut = theFrameInfo.time;
    }
    lastBallPosition = theTeamBallModel.position;
  }

  if(theGameInfo.state == STATE_PLAYING && theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) > 500 && static_cast<int>(theGameInfo.dropInTime) * 1000 < maxTimeToBallDropIn)
  {
    CROSS("module:GlobalFieldCoverageProvider:ballOutPosition",
          ballOutPosition.x(), ballOutPosition.y(), 75, 30,
          Drawings::solidPen, ColorRGBA(255, 192, 203));

    calculateDropInPosition(globalFieldCoverage);
    CROSS("module:GlobalFieldCoverageProvider:dropInPosition",
          globalFieldCoverage.ballDropInPosition.x(), globalFieldCoverage.ballDropInPosition.y(), 75, 30,
          Drawings::solidPen, ColorRGBA(255, 192, 203));

    const Vector2i dropInCell(static_cast<int>((globalFieldCoverage.ballDropInPosition.x() - theFieldDimensions.xPosOwnGroundline) / cellLengthX),
                              static_cast<int>((globalFieldCoverage.ballDropInPosition.y() - theFieldDimensions.yPosRightSideline) / cellLengthY));

    const int min = -static_cast<int>(Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosLeftSideline).squaredNorm()) / 1000;

    const int yOtherLine = numOfCellsY - 1 - dropInCell.y();

    const int otherLineTime = -sqr(6000) / 1000;
    const int dropInLineTime = otherLineTime - numOfCellsX * 1000;

    for(int y = 0; y < numOfCellsY; ++y)
    {
      if(y == dropInCell.y())
      {
        for(int x = 0; x < numOfCellsX; ++x)
        {
          globalFieldCoverage.grid[y * numOfCellsX + x].timestamp = theFrameInfo.time;
          globalFieldCoverage.grid[y * numOfCellsX + x].coverage = dropInLineTime + (std::abs(dropInCell.x() - x) * 1000);
        }
      }
      else if(y == yOtherLine)
      {
        for(int x = 0; x < numOfCellsX; ++x)
        {
          globalFieldCoverage.grid[y * numOfCellsX + x].timestamp = theFrameInfo.time;
          globalFieldCoverage.grid[y * numOfCellsX + x].coverage = otherLineTime;
        }
      }
      else
      {
        for(int x = 0; x < numOfCellsX; ++x)
        {
          globalFieldCoverage.grid[y * numOfCellsX + x].timestamp = theFrameInfo.time;
          globalFieldCoverage.grid[y * numOfCellsX + x].coverage = min + static_cast<int>(globalFieldCoverage.grid[y * numOfCellsX + x].positionOnField.squaredNorm()) / 1000;
        }
      }
    }
  }
}

void GlobalFieldCoverageProvider::calculateDropInPosition(GlobalFieldCoverage& globalFieldCoverage)
{
  const bool ownTeamKicked = theGameInfo.dropInTeam == theOwnTeamInfo.teamNumber;

  globalFieldCoverage.ballDropInPosition.y() = lastBallPositionLyingInsideField.y() < 0 ? theFieldDimensions.yPosRightDropInLine : theFieldDimensions.yPosLeftDropInLine;
  globalFieldCoverage.ballDropInPosition.x() = lastBallPositionLyingInsideField.x() + (ownTeamKicked ? -dropInPenaltyDistance : dropInPenaltyDistance);

  if(theFrameInfo.getTimeSince(lastTimeBallWentOut) < maxTimeToBallDropIn)
  {
    globalFieldCoverage.ballDropInPosition.y() = ballOutPosition.y() < 0 ? theFieldDimensions.yPosRightDropInLine : theFieldDimensions.yPosLeftDropInLine;

    if(ballOutPosition.x() > theFieldDimensions.xPosOpponentGroundline - 10.0f && ballOutPosition.x() < theFieldDimensions.xPosOpponentGroundline + 10.0f)
      globalFieldCoverage.ballDropInPosition.x() = ownTeamKicked ? std::min(globalFieldCoverage.ballDropInPosition.x(), 0.f) : theFieldDimensions.xPosOpponentDropInLine;
    else if(ballOutPosition.x() > theFieldDimensions.xPosOwnGroundline - 10.0f && ballOutPosition.x() < theFieldDimensions.xPosOwnGroundline + 10.0f)
      globalFieldCoverage.ballDropInPosition.x() = ownTeamKicked ? theFieldDimensions.xPosOwnDropInLine : std::max(globalFieldCoverage.ballDropInPosition.x(), 0.f);
    else
      globalFieldCoverage.ballDropInPosition.x() += ownTeamKicked ? -dropInPenaltyDistance : dropInPenaltyDistance;
  }

  globalFieldCoverage.ballDropInPosition.x() = clip(globalFieldCoverage.ballDropInPosition.x(), theFieldDimensions.xPosOwnDropInLine, theFieldDimensions.xPosOpponentDropInLine);
}

void GlobalFieldCoverageProvider::setCoverageAtFieldPosition(GlobalFieldCoverage& globalFieldCoverage, const Vector2f& positionOnField, const int coverage) const
{
  if(theFieldDimensions.isInsideField(positionOnField))
  {
    ASSERT(std::isfinite(positionOnField.x()));
    ASSERT(std::isfinite(positionOnField.y()));
    const int x = static_cast<int>((positionOnField.x() - theFieldDimensions.xPosOwnGroundline) / cellLengthX);
    const int y = static_cast<int>((positionOnField.y() - theFieldDimensions.yPosRightSideline) / cellLengthY);

    globalFieldCoverage.grid[y * numOfCellsX + x].timestamp = theFrameInfo.time;
    globalFieldCoverage.grid[y * numOfCellsX + x].coverage = coverage;
  }
}

void GlobalFieldCoverageProvider::init(GlobalFieldCoverage& globalFieldCoverage)
{
  initDone = true;

  cellLengthX = theFieldDimensions.xPosOpponentGroundline * 2 / numOfCellsX;
  cellLengthY = theFieldDimensions.yPosLeftSideline * 2 / numOfCellsY;

  float positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
  float positionOnFieldY = theFieldDimensions.yPosRightSideline + cellLengthY / 2.f;
  const unsigned time = std::max(10000u, theFrameInfo.time);

  globalFieldCoverage.grid.reserve(numOfCellsY * numOfCellsX);
  for(int y = 0; y < numOfCellsY; ++y)
  {
    for(int x = 0; x < numOfCellsX; ++x)
    {
      globalFieldCoverage.grid.emplace_back(time, time, positionOnFieldX, positionOnFieldY, cellLengthX, cellLengthY);
      positionOnFieldX += cellLengthX;
    }
    positionOnFieldX = theFieldDimensions.xPosOwnGroundline + cellLengthX / 2.f;
    positionOnFieldY += cellLengthY;
  }
}

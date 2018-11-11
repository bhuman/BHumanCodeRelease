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
  if(theGameInfo.competitionType == COMPETITION_TYPE_MIXEDTEAM)
    for(size_t y = 0; y < theHulkFieldCoverage.lines.size(); ++y)
      addLine(theHulkFieldCoverage.lines[y]);
  for(const auto& teammate : theTeamData.teammates)
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
  if(theGameInfo.state == STATE_PLAYING && theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen) > 500
     && theFrameInfo.getTimeSince(theBallDropInModel.lastTimeWhenBallWentOut) < maxTimeToBallDropIn)
  {
    const Vector2i dropInCell(static_cast<int>((theBallDropInModel.dropInPosition.x() - theFieldDimensions.xPosOwnGroundline) / cellLengthX),
                              static_cast<int>((theBallDropInModel.dropInPosition.y() - theFieldDimensions.yPosRightSideline) / cellLengthY));

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

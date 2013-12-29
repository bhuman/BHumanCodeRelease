/**
* @file RegionAnalyzer.h
* @author jeff
* @author Florian Maaß
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/CameraMatrix.h"
#include "Representations/Perception/RegionPercept.h"
#include "Representations/Perception/LineSpots.h"
#include "Representations/Perception/BallSpots.h"

MODULE(RegionAnalyzer)
  REQUIRES(CameraMatrix)
  REQUIRES(RegionPercept)
  PROVIDES_WITH_MODIFY_AND_DRAW(BallSpots)
  REQUIRES(BallSpots)
  PROVIDES_WITH_MODIFY_AND_DRAW(LineSpots)
  LOADS_PARAMETER(int, minLineSize) /**< The minimum size of a line region */
  LOADS_PARAMETER(int, minLineSegmentCount) /**< The minimum number of segments for a line regions */
  LOADS_PARAMETER(int, maxLineNghbNoneSize) /**< The maximum size of neighboring none colored regions for a line region */
  LOADS_PARAMETER(int, minLineNghbGreenAboveSize) /**< The minimum summed size of green segments above a line region */
  LOADS_PARAMETER(int, minLineNghbGreenBelowSize) /**< The minimum summed size of green segments below a line region */
  LOADS_PARAMETER(int, minLineNghbGreenSideSize) /**< The minimum summed size of green segments on each side of a line region */
  LOADS_PARAMETER(int, minLineSingleSegmentSize) /**< The minimum size a region needs to be a line if the minLineSegmentCount criterion is not met */
  LOADS_PARAMETER(float, maxLineNghbNoneRatio) /**< The maximum ratio of none neighbor regions for a line region */
  LOADS_PARAMETER(int, maxLineNghbGreySkip) /**< The maximal amount of pixels skip to find some green above/below/aside a white region */
  LOADS_PARAMETER(int, minRobotRegionSize) /**< The minimum region size for a white region to become nonLineSpot */
  LOADS_PARAMETER(float, maxRobotRegionAlphaDiff) /**< The maximum angle offset a white region can have from upright to become a nonLineSpot */
  LOADS_PARAMETER(float, minRobotWidthRatio) /**< The minimum height/width ratio a white regions needs to become nonLineSpot */
END_MODULE

/**
 * @class RegionAnalyzer
 * This class analyzes the regions from the RegionPercept and creates
 * LinesSpots, NonLineSpots and BallSpots.
 */
class RegionAnalyzer: public RegionAnalyzerBase
{
private:
  LineSpots* lineSpots; /**< The linespots found. */
  BallSpots* ballSpots;

  /** update the LineSpots Representation */
  void update(LineSpots& otherLineSpots);

  /** update the BallSpots Representation */
  void update(BallSpots& otherBallSpots){ballSpots = &otherBallSpots;};

  /** analyzes all the regions and create the lineSpots and nonLineSpots */
  void analyzeRegions();

  /** Checks whether a region is a line region
   * @param region the region to be checked
   * @param direction the direction of the spot is returned
   * @param spot a lineSpots for the region
   * @return is it a line?
   */
  inline bool isLine(const RegionPercept::Region* region, float& direction, LineSpots::LineSpot& spot);
  /** Returns how much green is below the region */
  int getGreenBelow(const RegionPercept::Region* region);
  /** Returns how much green is above the region */
  int getGreenAbove(const RegionPercept::Region* region);
  /** Returns how much green is left to the region */
  int getGreenLeft(const RegionPercept::Region* region);
  /** Returns how much green is right to the region */
  int getGreenRight(const RegionPercept::Region* region);
};

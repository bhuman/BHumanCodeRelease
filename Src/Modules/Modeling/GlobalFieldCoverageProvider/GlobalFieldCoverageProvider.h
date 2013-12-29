/**
* @file GlobalFieldCoverageProvider.h
* Declares a class that provides the global field coverage model.
* @author Felix Wenk
*/

#pragma once

#include "Tools/Module/Module.h"
#include "Tools/Math/Vector2.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Infrastructure/TeamMateData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/GlobalFieldCoverage.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/BehaviorControl/BehaviorControlOutput.h"

#include <stack>

MODULE(GlobalFieldCoverageProvider)
  REQUIRES(FieldCoverage)
  REQUIRES(TeamMateData)
  REQUIRES(FrameInfo)
  REQUIRES(RobotInfo)
  REQUIRES(RobotPose)
  REQUIRES(FieldDimensions)
  USES(BehaviorControlOutput)
  PROVIDES_WITH_MODIFY(GlobalFieldCoverage)
END_MODULE

/**
 * @class GlobalFieldCoverageProvider
 * Provides the global field coverage
 */
class GlobalFieldCoverageProvider : public GlobalFieldCoverageProviderBase
{
public:
  GlobalFieldCoverageProvider();
  /**
  * Provides the global field coverage representation
  */
  void update(GlobalFieldCoverage& globalFieldCoverage);

private:
  class Region
  {
  public:
    Vector2<> mean; /**< Mean in field coordinates */
    Vector2<int> acc; /**< Accumulated cell coordinates */
    int cells; /**< Number of cells of this region. */
    int robotNumber; /**< The number of the robot used to initialize the mean. */

    Region() : mean(0, 0), acc(0, 0), cells(0), robotNumber(-1) {}
    Region(const Vector2<>& rpose, int robotNumber) : mean(rpose), acc(0, 0), cells(0), robotNumber(robotNumber) {}

    /**
     * Squared distance between mean of this and other region in squared cells.
     */
    float distance2(const Region& other) const;

    /**
     * Squared distance between mean of this region and cell (x,y) in squared cells.
     */
    float distance2(int x, int y, const Vector2<>& cellLengths, float xPosOwnGroundline, float yPosRightSideline) const;

    void adjustMean(const Vector2<>& cellLengths, float xPosOwnGroundline, float yPosRightSideline);
  };

  /* Frequently used field dimensions as floats. */
  float xPosOwnGroundline;
  float yPosRightSideline;

  Vector2<> cellLengths; /** Length of a cell in x and y direction. */

  /**
   * Distance between the throw in line and the field border on the same side.
   */
  const float throwInLineDistance;

  /**
   * Number of cells between the throw in line and the field border on the same side.
   */
  int throwInLineCellOffset;

  /**
   * Accumulated field coverage grids of the other team mates.
   */
  unsigned fieldCoverageGrids[TeamMateData::numOfPlayers][FieldCoverage::GridInterval::xSteps* FieldCoverage::GridInterval::ySteps];

  /**
   * Partitioning of the field coverage grid used for exploring the field with
   * more than a one robot.
   */
  Region regions[TeamMateData::numOfPlayers];

  /**
   * Array to assign each cell to a region.
   */
  Region* cell2Region[FieldCoverage::GridInterval::xSteps* FieldCoverage::GridInterval::ySteps];

  /**
   * Largest components of the regions of each robot.
   */
  std::vector<int> largestComponent[TeamMateData::numOfPlayers];

  /**
   * Centers of each of the largest components.
   */
  Vector2<> componentCenter[TeamMateData::numOfPlayers];

  /**
   * Returns the local coverage value of robot 'robotId' of the cell 'cellIdx'.
   */
  unsigned char coverage(int robotId, int cellIdx) const;

  /**
   * Updates the field coverage grid of team mate 'teamMate'
   * with the data from the grid interval 'interval'.
   */
  void updateFieldCoverageGrids();

  /**
   * Merges the grids which were received via teamcomm into the
   * mergedGrid and returns the index of the worst covered cell.
   */
  int mergeGrids(GlobalFieldCoverage::Grid& mergedGrid);

  /**
   * Updates index of the worst covered cell in the coverage.
   * NOTE: This method is intended to be run after the coverage grid was updated.
   */
  void updateWorstCoveredCell(int worstIdx, GlobalFieldCoverage& globalFieldCoverage);

  /**
   * Calculates the center of the region 'region' and returns it in 'center'.
   */
  void calcCenterOfRegion(const std::vector<int>& region, Vector2<>& center) const;

  /**
   * Modifies the global field coverage grid 'grid' such that local field coverage grids
   */
  void ballThrowIn(GlobalFieldCoverage::Grid& grid);

  /**
   * Calculate covered/uncovered threshold from field coverage grid.
   *
   * Returns the coverage threshold value or -1 if no such value
   * has been found.
   */
  int findThreshold(const GlobalFieldCoverage& coverage);

  /**
   * Manage the regions used to generate patrol targets. There are only regions for robots
   * which are playing on the field. I.e. there's no region for the goalie, penalized players, etc.
   *
   * Returns the number of robots ready to patrol.
   */
  int initRegions();

  /**
   * Adds or updates the region for the robot with the number 'robotNumber'.
   */
  void updatePatrol(const Vector2<>& rpose, int robotNumber);

  /**
   * Removes the region of the robot with the number 'robotNumber'.
   */
  void removeFromPatrol(int robotNumber);

  /**
   * Finds the largest connected components within the regions.
   */
  void findLargestComponents();

  /**
   * Finds the component containing the cell (x,y).
   */
  void findComponent(int x, int y, bool marked[], std::vector<int>& component);

  /**
   * Finds field coverage regions to search in.
   */
  void findRegions(const GlobalFieldCoverage& coverage, int numPatrol);

  /**
   * Transforms field coordinates 'translation' to cell coordinates.
   */
  void field2Cell(Vector2<int>& cell, const Vector2<>& translation) const;

  /**
   * Produces a nice 3D drawing of the global field coverage.
   */
  void draw3D(const GlobalFieldCoverage& globalFieldCoverage) const;
};

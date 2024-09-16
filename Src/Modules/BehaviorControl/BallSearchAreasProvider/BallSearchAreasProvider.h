/**
 * @file BallSearchAreasProvider.h
 *
 * This file declares a module that describes the areas on the field where the ball should be searched.
 *
 * @author Sina Schreiber
 */

#pragma once

#include "Framework/Module.h"
#include "Math/Geometry.h"
#include "Representations/Modeling/BallDropInModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/BallSearchAreas.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Tools/Math/Projection.h"

MODULE(BallSearchAreasProvider,
{,
  REQUIRES(BallDropInModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(RobotPose),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ObstacleModel),
  PROVIDES(BallSearchAreas),
  DEFINES_PARAMETERS(
  {,
    (unsigned)(500) cellWidth, /**< the width of the cells */
    (unsigned)(500) cellHeight, /**< the height of the cells */
    (float)(3500.f) maxDistanceToCell, /**< the max distance to a cell. */
    (unsigned)(200) obstacleOffset, /**< offset to be added to the obstacle width in the sector wheel*/
    (unsigned char)(90) heatmapAlpha, /**< Transparency of the heatmap between 0 (invisible) and 255 (opaque) */
    (float)(6000.f) heatmapMaxTime, /**< The max time for the liniear interpolation of the heatmap. */
  }),
});

class BallSearchAreasProvider : public BallSearchAreasProviderBase
{
public:
  BallSearchAreasProvider();

private:
  unsigned cellCountX; /**< the number of the cells on field in x direction*/
  unsigned cellCountY; /**< the number of the cells on field in y direction*/
  std::vector<BallSearchAreas::Cell> grid; /**< the grid for the complete field*/

  const Geometry::Rect rectLeftOpponentCorner = Geometry::Rect(Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea), Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftTouchline)); /**< the rectangle for the left opponent corner*/
  const Geometry::Rect rectRightOpponentCorner = Geometry::Rect(Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosRightTouchline), Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea)); /**< the rectangle for the right opponent corner*/

  const Geometry::Rect rectLeftOwnCorner = Geometry::Rect(Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftTouchline), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea));
  const Geometry::Rect rectRightOwnCorner = Geometry::Rect(Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosRightGoalArea), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightTouchline));

  const Geometry::Rect rectOwnGoalArea = Geometry::Rect(Vector2f(theFieldDimensions.xPosOwnFieldBorder, theFieldDimensions.yPosLeftGoalArea), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea));/**<the rectangle for the own goal area*/
  const Geometry::Rect rectOpponentGoalArea = Geometry::Rect(Vector2f(theFieldDimensions.xPosOpponentFieldBorder, theFieldDimensions.yPosLeftGoalArea), Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea));/**<the rectangle for the own goal area*/

  std::vector<ColorRGBA> colorVector; /**< color vector for the heatmap*/

  /**
   * Initializes the grid.
   */
  void initializeGrid();

  /**
   * This method updates the ball search areas.
   * This method updates the cell position dynamically while the robot is moving by adding the odometer data to the position of the cell.
   * @param ballSearchAreas
   */
  void update(BallSearchAreas&) override;

  /**
   * Visualizes the ball search area with debug drawings.
   */
  void draw();

  /**
   * This method returns the cells within the Voronoi region of the given robot
   * @param agent the robot
   */
  const std::vector<BallSearchAreas::Cell> getVoronoiGrid(const Agent& agent) const;

  /**
   * This method updates the timestamp within the given cell.
   * @param the cell to be updated.
   */
  void updateTimestamp(BallSearchAreas::Cell& cell);

  /**
   * This method returns if a given cell is in the view of the robot
   * @param cell the cell to be checked if it is inside of the view of the robot
   */
  bool cellInView(const BallSearchAreas::Cell& cell, const std::vector<Vector2f>& p) const;

  /**
   * This method returns the squared distance between the robot and the cell.
   * @param cell the cell where the distance is checked
   */
  float returnDistanceToCellSqr(const BallSearchAreas::Cell& cell) const;

  /**
   * This method resets the timestamps of the obstacle grid, if the GameState is a
   * standard situation and it is sure that the will be replaced by a referee.
   */
  void reset();

  /**
   * This method calculates the cell within the voronoi grid the robot should search next.
   * Therefore the timestamp will be multiplied by the priority.
   * @param gridToSearch the grid to be filtered by the cell to be searched next
   */
  const Vector2f positionCellToSearchNext(const std::vector<BallSearchAreas::Cell>& gridToSearch) const;

  /**
   * This method creates a sectorwheel around the robot for detecting obstacles in the sight of the searching robot
   */
  std::list<SectorWheel::Sector> calculateObstacleSectors() const;

  /**
   * This method updates the timestamps within the sectors of the sector wheel depending on the type of the sector.
   * If the type is free the timestamp will be updated, if the sector is in sight of the robot.
   * If the type is obstacle, the section between the robot and the obstacle will be updated.
   */
  void updateCells();
};

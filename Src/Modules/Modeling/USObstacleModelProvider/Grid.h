/**
 * @file Grid.h
 * The file declares the base class for grids.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#include "Tools/Math/Pose2D.h"

/**
 * @class Grid
 * The base class for grids.
 */
class Grid : public Streamable
{
protected:
  /**
   * The method makes the object streamable.
   * @param in The stream from which the object is read (if in != 0).
   * @param out The stream to which the object is written (if out != 0).
   */
  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN;
    STREAM(width);
    STREAM(widthInCells);
    STREAM(heightInCells);
    STREAM(center);
    cellSize = widthInCells ? width / widthInCells : 0;
    height = heightInCells * cellSize;
    STREAM_REGISTER_FINISH;
  }

public:
  float width, /**< The width of the grid in mm. */
        height, /**< The height of the grid in mm. */
        cellSize; /**< The width and height of each single cell in mm. */
  int widthInCells, /**< The width of the grid in number of cells. */
      heightInCells; /**< The height of the grid in number of cells. */

  /**
   * The position of the center of the grid in world coordinates.
   * The center is defined as the corner between  widthInCells/2-1, heightInCells/2-1
   * and widthInCells/2, heightInCells/2.
   */
  Vector2<> center;

  /**
   * Constructor defining a grid without cells.
   */
  Grid();

  /**
   * Constructor defining the size of the grid.
   * @param width The width of the grid in mm.
   * @param widthInCells The width of the grid in number of cells.
   * @param heightInCells The height of the grid in number of cells.
   */
  Grid(float width, int widthInCells, int heightInCells);

  /**
   * The method returns whether cell 'pos' is within the range of the grid.
   * @param pos The position that is tested.
   * @return Is pos inside the grid?
   */
  bool isCellInside(const Vector2<int>& pos) const
  {return 0 <= pos.x && pos.x < widthInCells && 0 <= pos.y && pos.y < heightInCells;}

  /**
   * The function transforms world coordinates into grid coordinates.
   * @param worldPos The world position.
   * @return The corresponding position in grid coordinates. The position
   *         is not clipped to the size of the grid.
   */
  Vector2<int> worldToGrid(const Vector2<>& worldPos) const
  {
    return Vector2<int>(int(floor((worldPos.x - center.x) / cellSize + (widthInCells >> 1))),
                        int(floor((worldPos.y - center.y) / cellSize + (heightInCells >> 1))));
  }

  /**
   * The function transforms grid coordinates into world coordinates.
   * It returns the (-x/-y) corner of the cell in world coordinates.
   * @param gridPos The grid position.
   * @return The corresponding position in world coordinates.
   */
  Vector2<> gridToWorld(const Vector2<int>& gridPos) const
  {
    return Vector2<>(float(gridPos.x - (widthInCells >> 1)) * cellSize + center.x,
                     float(gridPos.y - (heightInCells >> 1)) * cellSize + center.y);
  }

  /**
   * The function transforms grid coordinates into world coordinates.
   * It returns the (-x/-y) corner of the cell in world coordinates.
   * @param gridPos The grid position.
   * @return The corresponding position in world coordinates.
   */
  Vector2<> gridToWorld(const Vector2<>& gridPos) const
  {
    return Vector2<>((gridPos.x - float(widthInCells >> 1)) * cellSize + center.x,
                     (gridPos.y - float(heightInCells >> 1)) * cellSize + center.y);
  }
};
